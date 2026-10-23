package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.shooter.shooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.turretSubsystem;

/** Simple driving, auto-aim, and shooter integration test. */
@TeleOp(name = "BioBuzz Auto Aim Test", group = "Testing")
public class biobuzzTeleOp extends LinearOpMode {

    /** Reduce this if the robot is too fast while testing the turret. */
    private static final double DRIVE_SPEED = 0.75;

    /** Mecanum wheels often need slightly more strafe command than forward command. */
    private static final double STRAFE_MULTIPLIER = 1.10;

    /** Manual turret power available on gamepad 1's D-pad left/right. */
    private static final double MANUAL_TURRET_POWER = 0.5;

    /** RPM added/subtracted by each D-pad up/down press in manual RPM tuning mode. */
    private static final double RPM_TUNING_STEP = 25.0;

    private boolean lastShooterToggleButton;
    private boolean lastRpmModeButton;
    private boolean lastDpadUp;
    private boolean lastDpadDown;
    private boolean manualRpmMode;
    private double manualTuningRpm = shooterSubsystem.SHOT_RPM[0];

    @Override
    public void runOpMode() {
        Hardware.resetInstance();
        Hardware robot = Hardware.getInstance(hardwareMap);
        turretSubsystem turret = new turretSubsystem(robot);
        shooterSubsystem shooter = new shooterSubsystem(robot);
        ElapsedTime loopTimer = new ElapsedTime();

        telemetry.addLine("BioBuzz auto-aim test ready");
        telemetry.addLine("G1 sticks: drive | hold RT: auto aim");
        telemetry.addLine("G1 dpad L/R: manual turret | A: shooter/intakes | B: stop");
        telemetry.addLine("G1 X: auto/manual RPM | manual mode dpad U/D: tune RPM");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            turret.stop();
            shooter.stop();
            return;
        }

        loopTimer.reset();
        try {
            while (opModeIsActive()) {
                double dtSeconds = loopTimer.seconds();
                loopTimer.reset();

                driveRobot(robot);
                updateShooterControls(shooter);

                boolean autoAim = gamepad1.right_trigger > 0.20;
                turret.setAutoAimEnabled(autoAim);
                double manualTurretCommand = (gamepad1.dpad_right ? 1.0 : 0.0)
                        - (gamepad1.dpad_left ? 1.0 : 0.0);
                turret.setManualPower(manualTurretCommand * MANUAL_TURRET_POWER);
                turret.update(dtSeconds);

                // Use predicted impact distance so RPM accounts for motion toward/away from target.
                if (manualRpmMode) {
                    shooter.setTargetRpm(manualTuningRpm);
                } else if (turret.hasDistance()) {
                    shooter.setTargetForDistance(turret.getImpactDistanceIn());
                }
                shooter.update(dtSeconds);

                boolean readyToShoot = shooter.isEnabled()
                        && turret.hasTarget()
                        && turret.isAimReady()
                        && shooter.isAtSpeed();

                addTelemetry(turret, shooter, autoAim, readyToShoot);
                telemetry.update();
            }
        } finally {
            stopDrive(robot);
            turret.stop();
            shooter.stop();
        }
    }

    private void driveRobot(Hardware robot) {
        double forward = applyDeadband(-gamepad1.left_stick_y) * DRIVE_SPEED;
        double strafe = applyDeadband(gamepad1.left_stick_x)
                * STRAFE_MULTIPLIER * DRIVE_SPEED;
        double turn = applyDeadband(gamepad1.right_stick_x) * DRIVE_SPEED;

        double denominator = Math.max(
                Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);
        robot.lf.setPower((forward + strafe + turn) / denominator);
        robot.lb.setPower((forward - strafe + turn) / denominator);
        robot.rf.setPower((forward - strafe - turn) / denominator);
        robot.rb.setPower((forward + strafe - turn) / denominator);
    }

    private void updateShooterControls(shooterSubsystem shooter) {
        boolean togglePressed = gamepad1.a;
        if (gamepad1.b) {
            shooter.setEnabled(false);
        } else if (togglePressed && !lastShooterToggleButton) {
            shooter.setEnabled(!shooter.isEnabled());
        }
        lastShooterToggleButton = togglePressed;

        if (gamepad1.x && !lastRpmModeButton) {
            manualRpmMode = !manualRpmMode;
            manualTuningRpm = shooter.getTargetRpm();
        }
        lastRpmModeButton = gamepad1.x;

        if (manualRpmMode) {
            if (gamepad1.dpad_up && !lastDpadUp) {
                manualTuningRpm = Math.min(
                        shooterSubsystem.MAX_RPM,
                        manualTuningRpm + RPM_TUNING_STEP);
            }
            if (gamepad1.dpad_down && !lastDpadDown) {
                manualTuningRpm = Math.max(0, manualTuningRpm - RPM_TUNING_STEP);
            }
        }
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
    }

    private void addTelemetry(
            turretSubsystem turret,
            shooterSubsystem shooter,
            boolean autoAim,
            boolean readyToShoot) {
        telemetry.addLine(readyToShoot
                ? ">>> READY TO SHOOT <<<"
                : "Not ready");

        telemetry.addData("Modes", "aim=%s, shooter=%s, rpm=%s",
                autoAim ? "AUTO" : "MANUAL",
                shooter.isEnabled() ? "ON" : "OFF",
                manualRpmMode ? "MANUAL" : "DISTANCE TABLE");
        telemetry.addData("Visible tag IDs", turret.getVisibleTagIds());
        telemetry.addData("Selected tag", turret.hasTarget()
                ? turret.getSelectedTagId() : "none");
        telemetry.addData("Vision", "tx=%+.2f deg, error=%+.2f deg, latency=%.0f ms",
                turret.getFilteredTxDeg(),
                turret.getLastVisionErrorDeg(),
                turret.getLastLatencyMs());
        telemetry.addData("Distance", "now=%.1f, impact=%.1f in",
                turret.getDistanceIn(), turret.getImpactDistanceIn());
        telemetry.addData("Prediction", "T=%.2fs, angleLead=%+.2f deg",
                turret.getPredictionTimeS(), turret.getPanLeadDeg());
        telemetry.addData("Target movement", "angle=%+.1f deg/s, range=%+.1f in/s",
                turret.getTargetRateDegPerSec(), turret.getRangeRateInPerSec());
        telemetry.addData("Odometry velocity", "forward=%+.1f, left=%+.1f in/s (%s)",
                turret.getRobotForwardVelocityInPerSec(),
                turret.getRobotLeftVelocityInPerSec(),
                turret.isOdometryRateValid() ? "valid" : "invalid");
        telemetry.addData("Odometry rates", "angle=%+.1f deg/s, range=%+.1f in/s",
                turret.getOdometryTargetRateDegPerSec(),
                turret.getOdometryRangeRateInPerSec());
        telemetry.addData("Turret", "ticks=%d, target=%.1f, power=%+.2f, ready=%s",
                turret.getCurrentTicks(),
                turret.getTargetTicks(),
                turret.getAppliedPower(),
                turret.isAimReady());
        telemetry.addData("Flywheel", "target=%.0f, command=%.0f, actual=%.0f RPM",
                shooter.getTargetRpm(),
                shooter.getCommandedRpm(),
                shooter.getActualRpm());
        telemetry.addData("Flywheel ready", shooter.isAtSpeed());
        telemetry.addLine("G1 RT aim | A shooter/intakes | B stop | X RPM mode");
        telemetry.addLine("Dpad L/R manual turret | manual RPM mode: dpad U/D");
    }

    private void stopDrive(Hardware robot) {
        robot.lf.setPower(0);
        robot.lb.setPower(0);
        robot.rf.setPower(0);
        robot.rb.setPower(0);
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < 0.05 ? 0 : value;
    }
}
