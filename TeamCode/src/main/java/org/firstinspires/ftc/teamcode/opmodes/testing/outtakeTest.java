package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp(name = "Outtake Test")
public class outtakeTest extends LinearOpMode {

    private static final double TICKS_PER_REV = 28.0;

    private double targetRPM = 400.0; // Default target RPM
    private static final double RPM_STEP_FINE = 50.0;    // D-Pad Up / Down (+/- 50 RPM)
    private static final double RPM_STEP_COARSE = 250.0; // D-Pad Right / Left (+/- 250 RPM)

    private static final double KP = 0.0020;
    private static final double KI = 0.0005;
    private static final double KD = 0.0001;
    private static final double KF = 0.000435;

    private double integralSum = 0.0;
    private double lastError = 0.0;
    private static final double MAX_INTEGRAL_SUM = 200.0;

    private static final double READY_TOLERANCE_RPM = 25.0;
    private int stableLoops = 0;
    private static final int REQUIRED_STABLE_LOOPS = 5;
    private boolean flywheelOn = false;
    private boolean lastX = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    private final ElapsedTime pidTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        Hardware robot = Hardware.getInstance(hardwareMap);
        DcMotorEx flywheel = robot.outtake;

        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Ready!");
        telemetry.update();

        waitForStart();
        pidTimer.reset();

        while (opModeIsActive()) {
            double dt = pidTimer.seconds();
            pidTimer.reset();
            if (dt <= 0 || dt > 0.2) {
                dt = 0.02;
            }

            boolean currentDpadUp = gamepad1.dpad_up;
            boolean currentDpadDown = gamepad1.dpad_down;
            boolean currentDpadLeft = gamepad1.dpad_left;
            boolean currentDpadRight = gamepad1.dpad_right;

            // Small RPM adjustments: D-Pad Up / Down (+/- 50 RPM)
            if (currentDpadUp && !lastDpadUp) {
                targetRPM += RPM_STEP_FINE;
            }
            if (currentDpadDown && !lastDpadDown) {
                targetRPM = Math.max(0, targetRPM - RPM_STEP_FINE);
            }

            // Large RPM adjustments: D-Pad Right / Left (+/- 250 RPM)
            if (currentDpadRight && !lastDpadRight) {
                targetRPM += RPM_STEP_COARSE;
            }
            if (currentDpadLeft && !lastDpadLeft) {
                targetRPM = Math.max(0, targetRPM - RPM_STEP_COARSE);
            }

            lastDpadUp = currentDpadUp;
            lastDpadDown = currentDpadDown;
            lastDpadLeft = currentDpadLeft;
            lastDpadRight = currentDpadRight;

            boolean currentX = gamepad1.x;
            if (currentX && !lastX) {
                flywheelOn = !flywheelOn; // X = ON / OFF
                if (!flywheelOn) {
                    resetPID();
                }
            }
            lastX = currentX;

            boolean shoot = flywheelOn;

            double targetTPS = shoot ? (targetRPM * TICKS_PER_REV / 60.0) : 0.0;
            double actualTPS = (flywheel != null) ? flywheel.getVelocity() : 0.0;
            double actualRPM = (actualTPS * 60.0) / TICKS_PER_REV;
            double errorTPS = targetTPS - actualTPS;
            double errorRPM = shoot ? (targetRPM - actualRPM) : 0.0;

            double motorPower = 0.0;

            if (shoot) {
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

            flywheel.setPower(motorPower);

            boolean isAtSpeed = shoot && (Math.abs(errorRPM) <= READY_TOLERANCE_RPM);
            if (isAtSpeed) {
                stableLoops++;
            } else {
                stableLoops = 0;
            }
            boolean readyToShoot = stableLoops >= REQUIRED_STABLE_LOOPS;

            telemetry.addData("Flywheel Toggle (X)", flywheelOn ? "ON (Running)" : "OFF (Stopped)");
            telemetry.addData("Flywheel Status", shoot
                    ? (readyToShoot ? "*** READY TO SHOOT ***" : "SPINNING UP (PID Control)")
                    : "STOPPED");

            telemetry.addLine();
            telemetry.addData("Target RPM", "%.0f RPM (%.1f TPS)", targetRPM, targetTPS);
            telemetry.addData("Actual RPM", "%.0f RPM (%.1f TPS)", actualRPM, actualTPS);
            telemetry.addData("RPM Error", "%.0f RPM", errorRPM);
            telemetry.addData("PID Motor Power", "%.1f%%", motorPower * 100.0);

            telemetry.addLine();
            telemetry.addLine("=== CONTROLS ===");
            telemetry.addLine("X Button: Toggle Flywheel ON / OFF");
            telemetry.addLine("D-Pad Up/Down: Small Adjust Target (+/- 50 RPM)");
            telemetry.addLine("D-Pad Right/Left: Large Adjust Target (+/- 250 RPM)");

            telemetry.update();
        }

        flywheel.setPower(0);
    }

    private void resetPID() {
        integralSum = 0.0;
        lastError = 0.0;
    }
}
