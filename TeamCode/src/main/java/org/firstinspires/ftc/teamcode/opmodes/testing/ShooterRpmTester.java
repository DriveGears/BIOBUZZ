package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.shooterSubsystem;

@TeleOp(name = "Shooter RPM Tester", group = "Testing")
public class ShooterRpmTester extends LinearOpMode {

    private static final double INTAKE_POWER = 1.0;
    private static final double TICKS_PER_REV = 28.0;
    private static final double KP = 0.0020;
    private static final double KF = 0.000435;

    @Override
    public void runOpMode() {
        Hardware.resetInstance();
        Hardware robot = Hardware.getInstance(hardwareMap);
        shooterSubsystem shooter = new shooterSubsystem(robot);
        ElapsedTime loopTimer = new ElapsedTime();

        double targetRpm = shooterSubsystem.COMPETITION_RPM;
        double targetTransferRpm = 1500.0;
        shooter.setTargetRpm(targetRpm);

        boolean intakeOn = false;
        boolean shooterOn = false;

        boolean lastRightTrigger = false;
        boolean lastLeftTrigger = false;
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;
        boolean lastDpadLeft = false;
        boolean lastDpadRight = false;

        telemetry.addLine("Shooter & Transfer RPM Tester Ready");
        telemetry.addLine("LT (tap): Toggle Shooter | RT (tap): Toggle Intake");
        telemetry.addLine("A (hold): Transfer | D-pad Up/Down: Shooter RPM | D-pad L/R: Transfer RPM");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            shooter.stop();
            return;
        }

        loopTimer.reset();

        try {
            while (opModeIsActive()) {
                robot.beginLoop();
                double dtSeconds = loopTimer.seconds();
                loopTimer.reset();

                // 1. Intake Toggle (RT)
                boolean currentRightTrigger = gamepad1.right_trigger > 0.5;
                if (currentRightTrigger && !lastRightTrigger) {
                    intakeOn = !intakeOn;
                }
                lastRightTrigger = currentRightTrigger;

                if (gamepad1.right_bumper) {
                    robot.intake.setPower(-INTAKE_POWER);
                } else {
                    robot.intake.setPower(intakeOn ? INTAKE_POWER : 0.0);
                }

                // 2. Shooter Toggle (LT)
                boolean currentLeftTrigger = gamepad1.left_trigger > 0.5;
                if (currentLeftTrigger && !lastLeftTrigger) {
                    shooterOn = !shooterOn;
                }
                lastLeftTrigger = currentLeftTrigger;

                shooter.setShooterEnabled(shooterOn);
                shooter.update(dtSeconds);

                // 3. Shooter RPM Adjustments (D-pad Up/Down +/- 100 RPM)
                boolean currentDpadUp = gamepad1.dpad_up;
                if (currentDpadUp && !lastDpadUp) {
                    targetRpm += 100.0;
                    shooter.setTargetRpm(targetRpm);
                }
                lastDpadUp = currentDpadUp;

                boolean currentDpadDown = gamepad1.dpad_down;
                if (currentDpadDown && !lastDpadDown) {
                    targetRpm = Math.max(0, targetRpm - 100.0);
                    shooter.setTargetRpm(targetRpm);
                }
                lastDpadDown = currentDpadDown;

                // 4. Transfer RPM Adjustments (D-pad Left/Right +/- 100 RPM)
                boolean currentDpadLeft = gamepad1.dpad_left;
                if (currentDpadLeft && !lastDpadLeft) {
                    targetTransferRpm = Math.max(0, targetTransferRpm - 100.0);
                }
                lastDpadLeft = currentDpadLeft;

                boolean currentDpadRight = gamepad1.dpad_right;
                if (currentDpadRight && !lastDpadRight) {
                    targetTransferRpm += 100.0;
                }
                lastDpadRight = currentDpadRight;

                // 5. Transfer Feed (A hold) with RPM PIDF
                boolean transferFeeding = gamepad1.a;
                double transferPower = 0.0;
                double actualTransferRpm = 0.0;
                if (transferFeeding) {
                    double targetTransferTps = targetTransferRpm * TICKS_PER_REV / 60.0;
                    double actualTransferTps = robot.transfer.getVelocity();
                    actualTransferRpm = (actualTransferTps * 60.0) / TICKS_PER_REV;
                    double errorTps = targetTransferTps - actualTransferTps;

                    transferPower = (KP * errorTps) + (KF * targetTransferTps);
                    transferPower = Math.max(-1.0, Math.min(1.0, transferPower));
                }
                robot.transfer.setPower(transferFeeding ? transferPower : 0.0);

                // Telemetry
                telemetry.addData("Shooter Status", shooterOn ? (shooter.isAtSpeed() ? "READY" : "SPINNING UP") : "OFF");
                telemetry.addData("Shooter Target RPM", "%.0f", targetRpm);
                telemetry.addData("Shooter Actual RPM", "%.0f", shooter.getActualRpm());
                telemetry.addLine();
                telemetry.addData("Transfer Status", transferFeeding ? "FEEDING" : "OFF");
                telemetry.addData("Transfer Target RPM", "%.0f", targetTransferRpm);
                telemetry.addData("Transfer Actual RPM", "%.0f", actualTransferRpm);
                telemetry.addData("Transfer Power", "%.2f", transferPower);
                telemetry.update();
            }
        } finally {
            shooter.stop();
            robot.intake.setPower(0);
            robot.transfer.setPower(0);
        }
    }
}
