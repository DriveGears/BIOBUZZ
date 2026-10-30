package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp(name = "Outtake & Transfer Encoder Test", group = "Testing")
public class outtakeTest2 extends LinearOpMode {

    private static final double TICKS_PER_REV = 28.0;

    private double targetFlywheelRPM = 2175.0; // Default target RPM for outtake
    private double targetTransferRPM = 1500.0; // Default target RPM for transfer
    private boolean adjustingTransfer = false; // Toggle between tuning outtake vs transfer with X button

    private static final double RPM_STEP_FINE = 50.0;    // D-Pad Up / Down (+/- 50 RPM)
    private static final double RPM_STEP_COARSE = 250.0; // D-Pad Right / Left (+/- 250 RPM)

    private static final double KP = 0.0020;
    private static final double KI = 0.0005;
    private static final double KD = 0.0001;
    private static final double KF = 0.000435;

    private double integralSum = 0.0;
    private double lastError = 0.0;
    private double transferIntegralSum = 0.0;
    private double transferLastError = 0.0;
    private static final double MAX_INTEGRAL_SUM = 200.0;

    private boolean flywheelOn = false;
    private boolean transferOn = false;

    private boolean lastA = false;
    private boolean lastY = false;
    private boolean lastX = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    private final ElapsedTime pidTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        Hardware.resetInstance();
        Hardware robot = Hardware.getInstance(hardwareMap);
        DcMotorEx flywheel = robot.outtake;
        DcMotorEx intake = robot.intake;
        DcMotorEx transfer = robot.transfer;

        if (flywheel != null) {
            flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (intake != null) {
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (transfer != null) {
            transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addLine("Outtake & Transfer Encoder Test Ready");
        telemetry.update();

        waitForStart();
        pidTimer.reset();

        while (opModeIsActive()) {
            double dt = pidTimer.seconds();
            pidTimer.reset();
            if (dt <= 0 || dt > 0.2) {
                dt = 0.02;
            }

            // Mecanum drive
            double forward = -applyDeadband(gamepad1.left_stick_y) * 0.75;
            double strafe = applyDeadband(gamepad1.left_stick_x) * 1.10 * 0.75;
            double turn = applyDeadband(gamepad1.right_stick_x) * 0.75;
            double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);
            robot.lf.setPower((forward + strafe + turn) / denominator);
            robot.lb.setPower((forward - strafe + turn) / denominator);
            robot.rf.setPower((forward - strafe - turn) / denominator);
            robot.rb.setPower((forward + strafe - turn) / denominator);

            boolean currentDpadUp = gamepad1.dpad_up;
            boolean currentDpadDown = gamepad1.dpad_down;
            boolean currentDpadLeft = gamepad1.dpad_left;
            boolean currentDpadRight = gamepad1.dpad_right;

            // X button: toggle tuning focus between Outtake and Transfer
            boolean currentX = gamepad1.x;
            if (currentX && !lastX) {
                adjustingTransfer = !adjustingTransfer;
            }
            lastX = currentX;

            // Small RPM adjustments: D-Pad Up / Down (+/- 50 RPM)
            if (currentDpadUp && !lastDpadUp) {
                if (adjustingTransfer) targetTransferRPM += RPM_STEP_FINE;
                else targetFlywheelRPM += RPM_STEP_FINE;
            }
            if (currentDpadDown && !lastDpadDown) {
                if (adjustingTransfer) targetTransferRPM = Math.max(0, targetTransferRPM - RPM_STEP_FINE);
                else targetFlywheelRPM = Math.max(0, targetFlywheelRPM - RPM_STEP_FINE);
            }

            // Large RPM adjustments: D-Pad Right / Left (+/- 250 RPM)
            if (currentDpadRight && !lastDpadRight) {
                if (adjustingTransfer) targetTransferRPM += RPM_STEP_COARSE;
                else targetFlywheelRPM += RPM_STEP_COARSE;
            }
            if (currentDpadLeft && !lastDpadLeft) {
                if (adjustingTransfer) targetTransferRPM = Math.max(0, targetTransferRPM - RPM_STEP_COARSE);
                else targetFlywheelRPM = Math.max(0, targetFlywheelRPM - RPM_STEP_COARSE);
            }

            lastDpadUp = currentDpadUp;
            lastDpadDown = currentDpadDown;
            lastDpadLeft = currentDpadLeft;
            lastDpadRight = currentDpadRight;

            // A button: toggle Outtake ON / OFF
            boolean currentA = gamepad1.a;
            if (currentA && !lastA) {
                flywheelOn = !flywheelOn;
                if (!flywheelOn) resetPID();
            }
            lastA = currentA;

            // Y button: toggle Transfer ON / OFF
            boolean currentY = gamepad1.y;
            if (currentY && !lastY) {
                transferOn = !transferOn;
                if (!transferOn) resetTransferPID();
            }
            lastY = currentY;

            if (gamepad1.b) {
                flywheelOn = false;
                transferOn = false;
                resetPID();
                resetTransferPID();
            }

            // Outtake PIDF
            double targetTPS = flywheelOn ? (targetFlywheelRPM * TICKS_PER_REV / 60.0) : 0.0;
            double actualTPS = (flywheel != null) ? flywheel.getVelocity() : 0.0;
            double actualRPM = (actualTPS * 60.0) / TICKS_PER_REV;
            double errorTPS = targetTPS - actualTPS;

            double motorPower = 0.0;
            if (flywheelOn) {
                double pTerm = KP * errorTPS;
                integralSum += errorTPS * dt;
                integralSum = Math.max(-MAX_INTEGRAL_SUM, Math.min(MAX_INTEGRAL_SUM, integralSum));
                double iTerm = KI * integralSum;
                double derivative = (errorTPS - lastError) / dt;
                double dTerm = KD * derivative;
                double fTerm = KF * targetTPS;

                motorPower = pTerm + iTerm + dTerm + fTerm;
                motorPower = Math.max(-1.0, Math.min(1.0, motorPower));
            } else {
                resetPID();
            }
            lastError = errorTPS;

            // Transfer PIDF
            double targetTransferTPS = transferOn ? (targetTransferRPM * TICKS_PER_REV / 60.0) : 0.0;
            double actualTransferTPS = (transfer != null) ? transfer.getVelocity() : 0.0;
            double actualTransferRPM = (actualTransferTPS * 60.0) / TICKS_PER_REV;
            double errorTransferTPS = targetTransferTPS - actualTransferTPS;

            double transferMotorPower = 0.0;
            if (transferOn) {
                double pTermT = KP * errorTransferTPS;
                transferIntegralSum += errorTransferTPS * dt;
                transferIntegralSum = Math.max(-MAX_INTEGRAL_SUM, Math.min(MAX_INTEGRAL_SUM, transferIntegralSum));
                double iTermT = KI * transferIntegralSum;
                double derivativeT = (errorTransferTPS - transferLastError) / dt;
                double dTermT = KD * derivativeT;
                double fTermT = KF * targetTransferTPS;

                transferMotorPower = pTermT + iTermT + dTermT + fTermT;
                transferMotorPower = Math.max(-1.0, Math.min(1.0, transferMotorPower));
            } else {
                resetTransferPID();
            }
            transferLastError = errorTransferTPS;

            if (flywheel != null) flywheel.setPower(motorPower);
            if (transfer != null) transfer.setPower(transferMotorPower);

            telemetry.addData("Tuning Focus (X)", adjustingTransfer ? "--> TRANSFER RPM" : "--> OUTTAKE RPM");
            telemetry.addData("Outtake Toggle (A)", flywheelOn ? "ON (Running)" : "OFF");
            telemetry.addData("Transfer Toggle (Y)", transferOn ? "ON (Running)" : "OFF");

            telemetry.addLine();
            telemetry.addData("Outtake Target RPM", "%.0f RPM", targetFlywheelRPM);
            telemetry.addData("Outtake Actual RPM", "%.0f RPM", actualRPM);
            telemetry.addData("Outtake Power", "%.1f%%", motorPower * 100.0);

            telemetry.addLine();
            telemetry.addData("Transfer Target RPM", "%.0f RPM", targetTransferRPM);
            telemetry.addData("Transfer Actual RPM", "%.0f RPM", actualTransferRPM);
            telemetry.addData("Transfer Power", "%.1f%%", transferMotorPower * 100.0);

            telemetry.addLine();
            telemetry.addLine("=== CONTROLS ===");
            telemetry.addLine("A Button: Toggle Outtake ON / OFF");
            telemetry.addLine("Y Button: Toggle Transfer ON / OFF");
            telemetry.addLine("X Button: Switch Tuning Target (Outtake vs Transfer)");
            telemetry.addLine("D-Pad Up/Down: +/- 50 RPM | Right/Left: +/- 250 RPM");
            telemetry.addLine("B Button: Stop All");

            telemetry.update();
        }

        if (flywheel != null) flywheel.setPower(0);
        if (transfer != null) transfer.setPower(0);
        robot.lf.setPower(0);
        robot.lb.setPower(0);
        robot.rf.setPower(0);
        robot.rb.setPower(0);
    }

    private void resetPID() {
        integralSum = 0.0;
        lastError = 0.0;
    }

    private void resetTransferPID() {
        transferIntegralSum = 0.0;
        transferLastError = 0.0;
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < 0.05 ? 0 : value;
    }
}
