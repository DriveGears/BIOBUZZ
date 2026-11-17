package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.shooterSubsystem;

/** Temporary live tuner for the production shooter controller. */
@Disabled
@TeleOp(name = "TEMP Shooter PIDF Tuner", group = "Testing")
public class outtakeTest extends LinearOpMode {

    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_POWER = 1.0;

    private final ElapsedTime loopTimer = new ElapsedTime();

    private Parameter selected = Parameter.TARGET_RPM;
    private boolean coarseSteps;
    private boolean intakeEnabled;
    private boolean lastRightTrigger;
    private boolean lastLeftTrigger;
    private boolean lastX;
    private boolean lastY;
    private boolean lastDpadLeft;
    private boolean lastDpadRight;
    private boolean lastDpadUp;
    private boolean lastDpadDown;

    private enum Parameter {
        TARGET_RPM,
        KF,
        KP,
        KI,
        KD
    }

    @Override
    public void runOpMode() {
        Hardware.resetInstance();
        // This standalone shooter test does not map or communicate with Limelight.
        Hardware robot = Hardware.getInstance(hardwareMap, false);
        shooterSubsystem shooter = new shooterSubsystem(robot);
        shooter.setTargetRpm(shooterSubsystem.COMPETITION_RPM);
        robot.intake.setPower(0);
        robot.transfer.setPower(0);
        robot.enableManualBulkReads();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("TEMP SHOOTER TUNER READY");
        telemetry.addLine("RT intake | RB reverse intake | LT flywheel | A transfer");
        telemetry.addLine("D-pad L/R selects; U/D adjusts; X toggles fine/coarse");
        telemetry.addLine("Y restores PIDF defaults; B stops everything");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            shooter.stop();
            return;
        }

        loopTimer.reset();
        lastRightTrigger = gamepad1.right_trigger > 0.5;
        lastLeftTrigger = gamepad1.left_trigger > 0.5;

        try {
            while (opModeIsActive()) {
                robot.beginLoop();
                double dtSeconds = loopTimer.seconds();
                loopTimer.reset();

                updateControls(robot, shooter);

                boolean feeding = gamepad1.a && !gamepad1.b;
                robot.transfer.setPower(feeding ? TRANSFER_POWER : 0);
                shooter.update(dtSeconds);

                double actualRpm = shooter.getActualRpm();
                addTelemetry(shooter, actualRpm, feeding);
                telemetry.update();
            }
        } finally {
            shooter.stop();
            robot.intake.setPower(0);
            robot.transfer.setPower(0);
        }
    }

    private void updateControls(Hardware robot, shooterSubsystem shooter) {
        boolean rightTrigger = gamepad1.right_trigger > 0.5;
        if (rightTrigger && !lastRightTrigger) {
            intakeEnabled = !intakeEnabled;
        }
        lastRightTrigger = rightTrigger;

        boolean leftTrigger = gamepad1.left_trigger > 0.5;
        if (leftTrigger && !lastLeftTrigger) {
            shooter.setShooterEnabled(!shooter.isEnabled());
        }
        lastLeftTrigger = leftTrigger;

        if (gamepad1.b) {
            intakeEnabled = false;
            shooter.setShooterEnabled(false);
        }

        if (gamepad1.b) {
            robot.intake.setPower(0);
        } else if (gamepad1.right_bumper) {
            robot.intake.setPower(-INTAKE_POWER);
        } else {
            robot.intake.setPower(intakeEnabled ? INTAKE_POWER : 0.0);
        }

        boolean x = gamepad1.x;
        if (x && !lastX) {
            coarseSteps = !coarseSteps;
        }
        lastX = x;

        boolean y = gamepad1.y;
        if (y && !lastY) {
            shooter.resetPidfGains();
        }
        lastY = y;

        boolean dpadLeft = gamepad1.dpad_left;
        if (dpadLeft && !lastDpadLeft) {
            selectPrevious();
        }
        lastDpadLeft = dpadLeft;

        boolean dpadRight = gamepad1.dpad_right;
        if (dpadRight && !lastDpadRight) {
            selectNext();
        }
        lastDpadRight = dpadRight;

        boolean dpadUp = gamepad1.dpad_up;
        if (dpadUp && !lastDpadUp) {
            adjustSelected(shooter, 1.0);
        }
        lastDpadUp = dpadUp;

        boolean dpadDown = gamepad1.dpad_down;
        if (dpadDown && !lastDpadDown) {
            adjustSelected(shooter, -1.0);
        }
        lastDpadDown = dpadDown;
    }

    private void selectPrevious() {
        Parameter[] values = Parameter.values();
        selected = values[(selected.ordinal() + values.length - 1) % values.length];
    }

    private void selectNext() {
        Parameter[] values = Parameter.values();
        selected = values[(selected.ordinal() + 1) % values.length];
    }

    private void adjustSelected(shooterSubsystem shooter, double direction) {
        double multiplier = coarseSteps ? 10.0 : 1.0;
        double kP = shooter.getKp();
        double kI = shooter.getKi();
        double kD = shooter.getKd();
        double kF = shooter.getKf();

        switch (selected) {
            case TARGET_RPM:
                shooter.setTargetRpm(shooter.getTargetRpm()
                        + direction * 25.0 * multiplier);
                return;
            case KP:
                kP += direction * 0.0001 * multiplier;
                break;
            case KI:
                kI += direction * 0.00005 * multiplier;
                break;
            case KD:
                kD += direction * 0.00001 * multiplier;
                break;
            case KF:
                kF += direction * 0.00001 * multiplier;
                break;
        }

        shooter.setPidfGains(kP, kI, kD, kF);
    }

    private void addTelemetry(
            shooterSubsystem shooter, double actualRpm, boolean feeding) {
        telemetry.addData("State", !shooter.isEnabled() ? "OFF"
                : shooter.isAtSpeed() ? "READY (green)" : "SPINNING (red)");
        telemetry.addData("Intake", gamepad1.right_bumper ? "REVERSE"
                : intakeEnabled ? "ON" : "OFF");
        telemetry.addData("Transfer", feeding ? "FEEDING" : "OFF");
        telemetry.addData("Step size", coarseSteps ? "COARSE (10x)" : "FINE (1x)");
        telemetry.addData("Selected", selected);
        // Numeric fields remain numeric so FTC Dashboard can plot them directly.
        telemetry.addData("targetRpm", shooter.getTargetRpm());
        telemetry.addData("actualRpm", actualRpm);
        telemetry.addData("errorRpm", shooter.getTargetRpm() - actualRpm);
        telemetry.addData("motorPower", shooter.getAppliedPower());
        telemetry.addData("KP", "%.6f%s", shooter.getKp(), marker(Parameter.KP));
        telemetry.addData("KI", "%.6f%s", shooter.getKi(), marker(Parameter.KI));
        telemetry.addData("KD", "%.6f%s", shooter.getKd(), marker(Parameter.KD));
        telemetry.addData("KF", "%.6f%s", shooter.getKf(), marker(Parameter.KF));
        telemetry.addData("Terms", "P=%+.3f I=%+.3f D=%+.3f F=%+.3f",
                shooter.getPTerm(), shooter.getITerm(),
                shooter.getDTerm(), shooter.getFTerm());
        telemetry.addLine("Dashboard graph: select targetRpm + actualRpm");
        telemetry.addLine("RT intake | RB reverse | LT flywheel | A transfer");
        telemetry.addLine("B stop everything | Y reset gains");
        telemetry.addLine("D-pad L/R select | U/D adjust | X fine/coarse");
        telemetry.addLine("Values are temporary: copy the final PIDF values into shooterSubsystem.");
    }

    private String marker(Parameter parameter) {
        return selected == parameter ? "  <" : "";
    }
}
