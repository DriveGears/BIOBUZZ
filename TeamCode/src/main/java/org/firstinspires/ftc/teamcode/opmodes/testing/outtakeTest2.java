package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.shooterSubsystem;

@TeleOp(name = "Outtake Encoder Test", group = "Testing")
public class outtakeTest2 extends LinearOpMode {

    private static final double DRIVE_SPEED = 0.75;
    private static final double STRAFE_MULTIPLIER = 1.10;
    private static final double RPM_TUNING_STEP = 50.0;

    private boolean lastShooterToggleButton = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private double manualTuningRpm = 1500.0; // default test RPM

    @Override
    public void runOpMode() {
        Hardware.resetInstance();
        Hardware robot = Hardware.getInstance(hardwareMap);
        shooterSubsystem shooter = new shooterSubsystem(robot);
        ElapsedTime loopTimer = new ElapsedTime();

        telemetry.addLine("Outtake Encoder Test Ready");
        telemetry.addLine("G1 Left Stick: Drive");
        telemetry.addLine("G1 A: Toggle Shooter ON/OFF | B: Stop");
        telemetry.addLine("G1 D-pad Up/Down: Adjust Target RPM (+/- 50)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
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

                shooter.setTargetRpm(manualTuningRpm);
                shooter.update(dtSeconds);

                telemetry.addData("Shooter Status", shooter.isEnabled() ? "ON (Running)" : "OFF (Stopped)");
                telemetry.addData("Target RPM", "%.0f RPM", shooter.getTargetRpm());
                telemetry.addData("Commanded RPM", "%.0f RPM", shooter.getCommandedRpm());
                telemetry.addData("Actual RPM (Encoder)", "%.0f RPM", shooter.getActualRpm());
                telemetry.addData("At Speed?", shooter.isAtSpeed() ? "*** READY ***" : "SPINNING UP");

                telemetry.addLine();
                telemetry.addLine("=== CONTROLS ===");
                telemetry.addLine("A Button: Toggle Shooter ON / OFF");
                telemetry.addLine("B Button: Stop Shooter");
                telemetry.addLine("D-Pad Up/Down: Adjust RPM (+/- 50)");

                telemetry.update();
            }
        } finally {
            stopDrive(robot);
            shooter.stop();
        }
    }

    private void driveRobot(Hardware robot) {
        double forward = applyDeadband(-gamepad1.left_stick_y) * DRIVE_SPEED;
        double strafe = applyDeadband(gamepad1.left_stick_x) * STRAFE_MULTIPLIER * DRIVE_SPEED;
        double turn = applyDeadband(gamepad1.right_stick_x) * DRIVE_SPEED;

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);
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

        if (gamepad1.dpad_up && !lastDpadUp) {
            manualTuningRpm = Math.min(shooterSubsystem.MAX_RPM, manualTuningRpm + RPM_TUNING_STEP);
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            manualTuningRpm = Math.max(0, manualTuningRpm - RPM_TUNING_STEP);
        }
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
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

//2600 - 2650 RPM NECTAR (CORNER)
//2175 - POLLEN (CORNER SHOT)

