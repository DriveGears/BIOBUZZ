package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp(name = "Flywheel Power Test", group = "Testing")
public class flywheelPowerTest extends LinearOpMode {

    private double targetPower = 0.5; // Default test power (0.0 to 1.0)
    private static final double POWER_STEP_FINE = 0.05;
    private static final double POWER_STEP_COARSE = 0.20;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;
    private boolean flywheelOn = false;
    private boolean lastX = false;

    @Override
    public void runOpMode() {
        Hardware robot = Hardware.getInstance(hardwareMap);
        DcMotorEx flywheel = robot.outtake;

        if (flywheel != null) {
            flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        telemetry.addData("Status", "Ready. Press X to toggle, D-pad to adjust power.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean currentDpadUp = gamepad1.dpad_up;
            boolean currentDpadDown = gamepad1.dpad_down;
            boolean currentDpadRight = gamepad1.dpad_right;
            boolean currentDpadLeft = gamepad1.dpad_left;

            // Power adjustments using D-Pad
            if (currentDpadUp && !lastDpadUp) {
                targetPower = Math.min(1.0, targetPower + POWER_STEP_FINE);
            }
            if (currentDpadDown && !lastDpadDown) {
                targetPower = Math.max(-1.0, targetPower - POWER_STEP_FINE);
            }
            if (currentDpadRight && !lastDpadRight) {
                targetPower = Math.min(1.0, targetPower + POWER_STEP_COARSE);
            }
            if (currentDpadLeft && !lastDpadLeft) {
                targetPower = Math.max(-1.0, targetPower - POWER_STEP_COARSE);
            }

            lastDpadUp = currentDpadUp;
            lastDpadDown = currentDpadDown;
            lastDpadRight = currentDpadRight;
            lastDpadLeft = currentDpadLeft;

            boolean currentX = gamepad1.x;
            if (currentX && !lastX) {
                flywheelOn = !flywheelOn; // Press X to toggle ON / OFF
            }
            lastX = currentX;

            double appliedPower = flywheelOn ? targetPower : 0.0;
            if (flywheel != null) {
                flywheel.setPower(appliedPower);
            }

            telemetry.addData("Flywheel (X)", flywheelOn ? "On" : "Off");
            telemetry.addData("Target Power", "%.2f (%.0f%%)", appliedPower, appliedPower * 100.0);
            telemetry.addLine();
            telemetry.addLine("Controls:");
            telemetry.addLine("X Button = Flywheel ON / OFF");
            telemetry.addLine("D-Pad Up/Down = Adjust Power (+/- 0.05)");
            telemetry.addLine("D-Pad Right/Left = Adjust Power (+/- 0.20)");

            telemetry.update();
        }

        if (flywheel != null) {
            flywheel.setPower(0);
        }
    }
}
