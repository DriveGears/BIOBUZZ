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

    private double targetFlywheelRPM = 400.0; // Default RPM for outtake
    private double targetTransferRPM = 300.0; // Default RPM for transfer
    private boolean adjustingTransfer = false; // Toggle between outtake vs transfer w A button

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

    private static final double READY_TOLERANCE_RPM = 25.0;
    private int stableLoops = 0;
    private static final int REQUIRED_STABLE_LOOPS = 5;

    private boolean flywheelOn = false;
    private boolean intakeOn = false;
    private boolean transferOn = false;

    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastA = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    private final ElapsedTime pidTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
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

            // Toggle between Outtake and Transfer using A
            boolean currentA = gamepad1.a;
            if (currentA && !lastA) {
                adjustingTransfer = !adjustingTransfer;
            }
            lastA = currentA;

            // Small RPM adjustments: D-Pad Up / Down (+/- 50 RPM)
            if (currentDpadUp && !lastDpadUp) {
                if (adjustingTransfer) {
                    targetTransferRPM += RPM_STEP_FINE;
                } else {
                    targetFlywheelRPM += RPM_STEP_FINE;
                }
            }
            if (currentDpadDown && !lastDpadDown) {
                if (adjustingTransfer) {
                    targetTransferRPM = Math.max(0, targetTransferRPM - RPM_STEP_FINE);
                } else {
                    targetFlywheelRPM = Math.max(0, targetFlywheelRPM - RPM_STEP_FINE);
                }
            }

            // Large RPM adjustments: D-Pad Right / Left (+/- 250 RPM)
            if (currentDpadRight && !lastDpadRight) {
                if (adjustingTransfer) {
                    targetTransferRPM += RPM_STEP_COARSE;
                } else {
                    targetFlywheelRPM += RPM_STEP_COARSE;
                }
            }
            if (currentDpadLeft && !lastDpadLeft) {
                if (adjustingTransfer) {
                    targetTransferRPM = Math.max(0, targetTransferRPM - RPM_STEP_COARSE);
                } else {
                    targetFlywheelRPM = Math.max(0, targetFlywheelRPM - RPM_STEP_COARSE);
                }
            }

            lastDpadUp = currentDpadUp;
            lastDpadDown = currentDpadDown;
            lastDpadLeft = currentDpadLeft;
            lastDpadRight = currentDpadRight;

            boolean currentX = gamepad1.x;
            if (currentX && !lastX) {
                flywheelOn = !flywheelOn; // X = Flywheel ON / OFF
                if (!flywheelOn) {
                    resetPID();
                }
            }
            lastX = currentX;

            boolean currentY = gamepad1.y;
            if (currentY && !lastY) {
                intakeOn = !intakeOn;     // Y = Intake ON / OFF
                transferOn = !transferOn; // Y = Transfer ON / OFF
                if (!transferOn) {
                    resetTransferPID();
                }
            }
            lastY = currentY;

            boolean shoot = flywheelOn;
            boolean transferRun = transferOn;

            // Outtake Flywheel PIDF Control
            double targetTPS = shoot ? (targetFlywheelRPM * TICKS_PER_REV / 60.0) : 0.0;
            double actualTPS = (flywheel != null) ? flywheel.getVelocity() : 0.0;
            double actualRPM = (actualTPS * 60.0) / TICKS_PER_REV;
            double errorTPS = targetTPS - actualTPS;
            double errorRPM = shoot ? (targetFlywheelRPM - actualRPM) : 0.0;

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

            // Transfer Motor PIDF Control
            double targetTransferTPS = transferRun ? (targetTransferRPM * TICKS_PER_REV / 60.0) : 0.0;
            double actualTransferTPS = (transfer != null) ? transfer.getVelocity() : 0.0;
            double actualTransferRPM = (actualTransferTPS * 60.0) / TICKS_PER_REV;
            double errorTransferTPS = targetTransferTPS - actualTransferTPS;
            double errorTransferRPM = transferRun ? (targetTransferRPM - actualTransferRPM) : 0.0;

            double transferMotorPower = 0.0;
            if (transferRun) {
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

            if (flywheel != null) {
                flywheel.setPower(motorPower);
            }

            if (intake != null) {
                intake.setPower(intakeOn ? 1.0 : 0.0);
            }

            if (transfer != null) {
                transfer.setPower(transferMotorPower);
            }

            boolean isAtSpeed = shoot && (Math.abs(errorRPM) <= READY_TOLERANCE_RPM);
            if (isAtSpeed) {
                stableLoops++;
            } else {
                stableLoops = 0;
            }
            boolean readyToShoot = stableLoops >= REQUIRED_STABLE_LOOPS;

            telemetry.addData("Tuning Focus (A)", adjustingTransfer ? "--> TRANSFER RPM" : "--> OUTTAKE (Flywheel) RPM");
            telemetry.addData("Flywheel Toggle (X)", flywheelOn ? "ON (Running)" : "OFF (Stopped)");
            telemetry.addData("Intake & Transfer Toggle (Y)", intakeOn ? "ON (Running)" : "OFF (Stopped)");
            telemetry.addData("Flywheel Status", shoot
                    ? (readyToShoot ? "*** READY TO SHOOT ***" : "SPINNING UP (PID Control)")
                    : "STOPPED");

            telemetry.addLine();
            telemetry.addData("Outtake Target RPM", "%.0f RPM (%.1f TPS)", targetFlywheelRPM, targetTPS);
            telemetry.addData("Outtake Actual RPM", "%.0f RPM (%.1f TPS)", actualRPM, actualTPS);
            telemetry.addData("Outtake RPM Error", "%.0f RPM", errorRPM);
            telemetry.addData("Outtake Power", "%.1f%%", motorPower * 100.0);

            telemetry.addLine();
            telemetry.addData("Transfer Target RPM", "%.0f RPM (%.1f TPS)", targetTransferRPM, targetTransferTPS);
            telemetry.addData("Transfer Actual RPM", "%.0f RPM (%.1f TPS)", actualTransferRPM, actualTransferTPS);
            telemetry.addData("Transfer RPM Error", "%.0f RPM", errorTransferRPM);
            telemetry.addData("Transfer Power", "%.1f%%", transferMotorPower * 100.0);

            telemetry.addLine();
            telemetry.addLine("=== CONTROLS ===");
            telemetry.addLine("X Button: Toggle Flywheel ON / OFF");
            telemetry.addLine("Y Button: Toggle Intake & Transfer ON / OFF");
            telemetry.addLine("A Button: Switch D-Pad Tuning Target (Outtake vs Transfer)");
            telemetry.addLine("D-Pad Up/Down: Small Adjust Target (+/- 50 RPM)");
            telemetry.addLine("D-Pad Right/Left: Large Adjust Target (+/- 250 RPM)");

            telemetry.update();
        }

        if (flywheel != null) {
            flywheel.setPower(0);
        }
        if (intake != null) {
            intake.setPower(0);
        }
        if (transfer != null) {
            transfer.setPower(0);
        }
    }

    private void resetPID() {
        integralSum = 0.0;
        lastError = 0.0;
    }

    private void resetTransferPID() {
        transferIntegralSum = 0.0;
        transferLastError = 0.0;
    }
}
