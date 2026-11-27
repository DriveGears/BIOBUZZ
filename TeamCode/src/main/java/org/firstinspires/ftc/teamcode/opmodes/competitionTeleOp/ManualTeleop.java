package org.firstinspires.ftc.teamcode.opmodes.competitionTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.shooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.turretSubsystem;

/** Competition fallback TeleOp that does not map or use the Limelight. */
@TeleOp(name = "MANUAL (NO LIMELIGHT) TeleOp", group = "Competition")
public class ManualTeleop extends LinearOpMode {

    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_POWER = 1.0;
    private static final double MANUAL_TURRET_POWER = 0.40;
    private static final double DRIVE_STICK_DEADBAND = 0.05;

    private boolean intakeToggle;
    private boolean shooterToggle;
    private boolean lastRightTrigger;
    private boolean lastLeftTrigger;
    private boolean lastTurretZeroButton;

    @Override
    public void runOpMode() {
        Hardware.resetInstance();
        // false deliberately avoids looking for a Limelight in the hardware map.
        Hardware robot = Hardware.getInstance(hardwareMap, false);
        shooterSubsystem shooter = new shooterSubsystem(robot);
        ElapsedTime loopTimer = new ElapsedTime();

        stopMechanisms(robot, shooter);
        robot.shooterLed.setPwmEnable();
        robot.shooterLed.setPosition(turretSubsystem.LED_MANUAL_POSITION);
        robot.enableManualBulkReads();
        telemetry.setMsTransmissionInterval(100);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Manual fallback ready - Limelight is not used");
            telemetry.addLine("Start with the turret mechanically centered");
            addControlsTelemetry();
            telemetry.update();
            idle();
        }

        if (isStopRequested()) {
            stopMechanisms(robot, shooter);
            return;
        }

        loopTimer.reset();
        lastRightTrigger = gamepad1.right_trigger > 0.5;
        lastLeftTrigger = gamepad1.left_trigger > 0.5;
        lastTurretZeroButton = gamepad1.start;

        try {
            while (opModeIsActive()) {
                robot.beginLoop();
                double dtSeconds = loopTimer.seconds();
                loopTimer.reset();

                drive(robot);
                updateIntake(robot);

                boolean currentLeftTrigger = gamepad1.left_trigger > 0.5;
                if (currentLeftTrigger && !lastLeftTrigger) {
                    shooterToggle = !shooterToggle;
                }
                lastLeftTrigger = currentLeftTrigger;

                // With no camera range, always use the tested competition fallback RPM.
                shooter.setTargetRpm(shooterSubsystem.COMPETITION_RPM);
                shooter.setShooterEnabled(shooterToggle);
                shooter.update(dtSeconds);

                if (gamepad1.a) {
                    robot.transfer.setPower(TRANSFER_POWER);
                } else if (gamepad1.x) {
                    robot.transfer.setPower(-TRANSFER_POWER);
                } else {
                    robot.transfer.setPower(0);
                }

                boolean turretZeroPressed = gamepad1.start;
                if (turretZeroPressed && !lastTurretZeroButton) {
                    robot.turret.setPower(0);
                    robot.setTurretCurrentPositionAsZero();
                }
                lastTurretZeroButton = turretZeroPressed;

                double turretCommand = (gamepad1.dpad_right ? 1.0 : 0.0)
                        - (gamepad1.dpad_left ? 1.0 : 0.0);
                int turretTicks = robot.getTurretPositionTicks();
                double turretPower = limitTurretAtSoftStops(
                        turretCommand * MANUAL_TURRET_POWER, turretTicks);
                robot.turret.setPower(turretPower);

                telemetry.addData("Mode", "MANUAL - NO LIMELIGHT");
                telemetry.addData("Turret", "ticks=%d power=%+.2f limits=[%d, %d]",
                        turretTicks, turretPower,
                        turretSubsystem.MIN_TURRET_TICKS,
                        turretSubsystem.MAX_TURRET_TICKS);
                telemetry.addData("Turret Encoder", "relative=%d = raw %d - saved zero %d",
                        turretTicks, robot.getTurretRawEncoderTicks(),
                        robot.getTurretEncoderZeroTicks());
                boolean hardLimitFault = turretTicks < turretSubsystem.MIN_TURRET_TICKS
                        - turretSubsystem.HARD_LIMIT_OVERRUN_TICKS
                        || turretTicks > turretSubsystem.MAX_TURRET_TICKS
                        + turretSubsystem.HARD_LIMIT_OVERRUN_TICKS;
                telemetry.addData("Turret Safety", hardLimitFault
                        ? "HARD LIMIT: manual movement allowed only toward center, then START"
                        : "relative limits OK");
                telemetry.addData("Intake", intakeToggle ? "ON" : "OFF");
                telemetry.addData("Shooter RPM", "target=%.0f actual=%.0f %s",
                        shooter.getTargetRpm(), shooter.getActualRpm(),
                        !shooterToggle ? "OFF"
                                : shooter.isAtSpeed() ? "READY" : "SPINNING UP");
                telemetry.addData("Transfer", gamepad1.a ? "FORWARD"
                        : gamepad1.x ? "REVERSE" : "OFF");
                addControlsTelemetry();
                telemetry.update();
            }
        } finally {
            robot.lf.setPower(0);
            robot.lb.setPower(0);
            robot.rf.setPower(0);
            robot.rb.setPower(0);
            stopMechanisms(robot, shooter);
        }
    }

    private void drive(Hardware robot) {
        double forward = -applyDeadband(gamepad1.left_stick_y);
        double lateral = applyDeadband(gamepad1.left_stick_x);
        double turn = applyDeadband(gamepad1.right_stick_x);
        double denominator = Math.max(
                Math.abs(forward) + Math.abs(lateral) + Math.abs(turn), 1.0);

        robot.lf.setPower((forward + lateral + turn) / denominator);
        robot.lb.setPower((forward - lateral + turn) / denominator);
        robot.rf.setPower((forward - lateral - turn) / denominator);
        robot.rb.setPower((forward + lateral - turn) / denominator);
    }

    private void updateIntake(Hardware robot) {
        boolean currentRightTrigger = gamepad1.right_trigger > 0.5;
        if (currentRightTrigger && !lastRightTrigger) {
            intakeToggle = !intakeToggle;
        }
        lastRightTrigger = currentRightTrigger;

        robot.intake.setPower(gamepad1.right_bumper
                ? -INTAKE_POWER
                : intakeToggle ? INTAKE_POWER : 0.0);
    }

    private static double limitTurretAtSoftStops(double power, int currentTicks) {
        double tickDirection = turretSubsystem.MOTOR_DIRECTION_SIGN * power;
        if (currentTicks < turretSubsystem.MIN_TURRET_TICKS
                - turretSubsystem.HARD_LIMIT_OVERRUN_TICKS) {
            return tickDirection > 0 ? power : 0.0;
        }
        if (currentTicks > turretSubsystem.MAX_TURRET_TICKS
                + turretSubsystem.HARD_LIMIT_OVERRUN_TICKS) {
            return tickDirection < 0 ? power : 0.0;
        }
        if (currentTicks >= turretSubsystem.MAX_TURRET_TICKS && tickDirection > 0) {
            return 0.0;
        }
        if (currentTicks <= turretSubsystem.MIN_TURRET_TICKS && tickDirection < 0) {
            return 0.0;
        }
        return power;
    }

    private static double applyDeadband(double value) {
        return Math.abs(value) < DRIVE_STICK_DEADBAND ? 0.0 : value;
    }

    private static void stopMechanisms(Hardware robot, shooterSubsystem shooter) {
        shooter.stop();
        robot.intake.setPower(0);
        robot.transfer.setPower(0);
        robot.turret.setPower(0);
    }

    private void addControlsTelemetry() {
        telemetry.addLine();
        telemetry.addLine("=== GAMEPAD 1 CONTROLS ===");
        telemetry.addLine("Left Stick: Drive/strafe | Right Stick X: Turn");
        telemetry.addLine("D-pad L/R (hold): Move turret manually");
        telemetry.addLine("START: set current turret position to 0");
        telemetry.addLine("Zero only while the turret is mechanically centered");
        telemetry.addLine("RT (tap): Toggle intake | RB (hold): Reverse intake");
        telemetry.addLine("LT (tap): Toggle flywheel | A (hold): Transfer forward");
        telemetry.addLine("X (hold): Reverse transfer");
        telemetry.addLine("LED: blue=manual turret mode");
    }
}
