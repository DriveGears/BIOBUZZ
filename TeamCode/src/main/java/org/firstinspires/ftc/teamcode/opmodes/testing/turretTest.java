package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.shooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.turretSubsystem;

@TeleOp(name = "biobuzz testing teleop", group = "Testing")
public class turretTest extends LinearOpMode {

    private static final double DRIVE_SPEED = 0.80;
    private static final double STRAFE_MULTIPLIER = 1.10;
    private static final double MANUAL_TURRET_POWER = 0.25;

    private boolean lastShooterButton = false;
    private boolean lastTurretZeroButton = false;

    @Override
    public void runOpMode() {
        Hardware.resetInstance();
        Hardware robot = Hardware.getInstance(hardwareMap);
        turretSubsystem turret = new turretSubsystem(robot);
        shooterSubsystem shooter = new shooterSubsystem(robot);
        ElapsedTime loopTimer = new ElapsedTime();

        telemetry.addLine("BioBuzz TeleOp Ready");
        telemetry.addLine("G1 Left Stick: Drive | Right Trigger: Hold Auto-Aim");
        telemetry.addLine("G1 D-pad L/R: Manual Turret | A: Toggle Shooter/Intakes | B: Stop");
        telemetry.addLine("Click BOTH sticks: set current turret angle to zero");
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

                // 1. Mecanum Drive
                double forward = -applyDeadband(gamepad1.left_stick_y) * DRIVE_SPEED;
                double strafe = applyDeadband(gamepad1.left_stick_x) * STRAFE_MULTIPLIER * DRIVE_SPEED;
                double turn = applyDeadband(gamepad1.right_stick_x) * DRIVE_SPEED;

                double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);
                robot.lf.setPower((forward + strafe + turn) / denominator);
                robot.lb.setPower((forward - strafe + turn) / denominator);
                robot.rf.setPower((forward - strafe - turn) / denominator);
                robot.rb.setPower((forward + strafe - turn) / denominator);

                // 2. Shooter / Outtake Controls (A to toggle, B to stop)
                boolean aPressed = gamepad1.a;
                if (gamepad1.b) {
                    shooter.setEnabled(false);
                } else if (aPressed && !lastShooterButton) {
                    shooter.setEnabled(!shooter.isEnabled());
                }
                lastShooterButton = aPressed;

                // Set target RPM for distance if turret has distance, otherwise default
                if (turret.hasDistance()) {
                    shooter.setTargetForDistance(turret.getImpactDistanceIn());
                } else {
                    shooter.setTargetRpm(shooterSubsystem.COMPETITION_RPM);
                }
                shooter.update(dtSeconds);

                // 3. Turret Auto-Aim & Manual Control
                boolean turretZeroPressed = gamepad1.left_stick_button
                        && gamepad1.right_stick_button;
                if (turretZeroPressed && !lastTurretZeroButton) {
                    turret.zeroAtCurrentPosition();
                }
                lastTurretZeroButton = turretZeroPressed;

                // Do not allow auto aim in the same loop as a zero operation.
                boolean autoAim = gamepad1.right_trigger > 0.20 && !turretZeroPressed;
                double manualTurret = (gamepad1.dpad_right ? 1.0 : 0.0) - (gamepad1.dpad_left ? 1.0 : 0.0);
                turret.update(dtSeconds, autoAim,
                        manualTurret * MANUAL_TURRET_POWER);

                // 4. Telemetry
                telemetry.addData("Drive", "Fwd=%.2f, Strafe=%.2f, Turn=%.2f", forward, strafe, turn);
                telemetry.addData("Turret Mode", autoAim ? "AUTO-AIM" : "MANUAL");
                telemetry.addData("Turret Tune", turretSubsystem.TUNE_VERSION);
                telemetry.addData("Target Visible", turret.hasTarget()
                        ? "YES (" + turret.getVisibleTagCount() + " tags: "
                          + turret.getVisibleTagIds() + ")"
                        : turret.isOdometryTrackingActive()
                          ? "ODOMETRY (vision lost)" : "NO");
                telemetry.addData("Odo Target", "active=%s age=%.0f ms bearing=%+.1f dist=%.1f",
                        turret.isOdometryTrackingActive(), turret.getTargetAgeMs(),
                        turret.getOdometryTrackedBearingDeg(),
                        turret.getOdometryTrackedDistanceIn());
                telemetry.addData("Cluster Center", turret.isClusterGeometryValid()
                        ? "GEOMETRY SOLVED" : "ANGULAR FALLBACK");
                telemetry.addData("Aim Ready?", turret.isAimReady() ? "READY" : "ALIGNING");
                telemetry.addData("Turret Position", "%d -> %.1f ticks [%d, %d]",
                        turret.getCurrentTicks(), turret.getTargetTicks(),
                        turretSubsystem.MIN_TURRET_TICKS,
                        turretSubsystem.MAX_TURRET_TICKS);
                telemetry.addData("Turret Raw/Zero", "%d / %d",
                        robot.getTurretRawEncoderTicks(),
                        robot.getTurretEncoderZeroTicks());
                telemetry.addData("Turret Limit",
                        turret.getCurrentTicks() <= turretSubsystem.MIN_TURRET_TICKS
                                ? "LEFT SOFT LIMIT"
                                : turret.getCurrentTicks() >= turretSubsystem.MAX_TURRET_TICKS
                                  ? "RIGHT SOFT LIMIT" : "clear");
                telemetry.addData("Turret PID", "err=%+.1f tk, vel=%+.1f tk/s, pwr=%+.3f, I=%+.3f",
                        turret.getPositionErrorTicks(),
                        turret.getFilteredMotorVelocityTicksPerSecond(),
                        turret.getAppliedPower(),
                        turret.getIntegralPower());
                telemetry.addData("Turret Vision", "tx=%+.2f deg, err=%+.2f deg",
                        turret.getFilteredTxDeg(), turret.getLastVisionErrorDeg());
                telemetry.addData("Shooter", "Enabled=%s, Actual RPM=%.0f", shooter.isEnabled() ? "ON" : "OFF", shooter.getActualRpm());
                telemetry.update();
            }
        } finally {
            robot.lf.setPower(0);
            robot.lb.setPower(0);
            robot.rf.setPower(0);
            robot.rb.setPower(0);
            turret.stop();
            shooter.stop();
        }
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < 0.05 ? 0 : value;
    }
}
