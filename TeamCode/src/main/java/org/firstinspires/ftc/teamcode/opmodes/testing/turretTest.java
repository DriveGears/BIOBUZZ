package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.FlightRecorder;
import org.firstinspires.ftc.teamcode.subsystems.shooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.turretSubsystem;

@TeleOp(name = "biobuzz turret testing teleop", group = "Testing")
public class turretTest extends LinearOpMode {

    private static final double DRIVE_SPEED = 0.80;
    private static final double STRAFE_MULTIPLIER = 1.10;
    private static final double MANUAL_TURRET_POWER = 0.40;
    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_POWER = 1.0;

    private boolean lastShooterButton = false;
    private boolean lastTurretZeroButton = false;
    private boolean lastFusionToggle = false;

    @Override
    public void runOpMode() {
        Hardware.resetInstance();
        Hardware robot = Hardware.getInstance(hardwareMap);
        turretSubsystem turret = new turretSubsystem(robot);
        shooterSubsystem shooter = new shooterSubsystem(robot);
        ElapsedTime loopTimer = new ElapsedTime();
        robot.intake.setPower(0);
        robot.transfer.setPower(0);

        FlightRecorder rec = FlightRecorder.start("testteleop");
        java.util.Map<String, String> cfg = turretSubsystem.configSnapshot();
        cfg.put("opmode", "turret_biobuzzTestTeleOp");
        cfg.put("tune_version", turretSubsystem.TUNE_VERSION);
        rec.writeConfig(cfg);
        turret.setRecorder(rec);
        rec.event("--- ODOMETRY INIT ---\n" + robot.getOdometryReport());
        robot.enableManualBulkReads();
        telemetry.setMsTransmissionInterval(100);

        for (String line : robot.getOdometryReport().split("\n")) {
            if (line.startsWith("ODO_CONFIGURED") || line.startsWith("ODO_WARNING")
                    || line.startsWith("ODO_LEAN") || line.startsWith("ODO_FIRMWARE")) telemetry.addLine(line);
        }
        telemetry.addLine("BioBuzz TeleOp Ready");
        telemetry.addLine("G1 Left Stick: Drive | Right Trigger: Hold Auto-Aim");
        telemetry.addLine("G1 D-pad L/R: Manual Turret | A: Toggle Shooter/Intakes | B: Stop");
        telemetry.addLine("Click BOTH sticks: set current turret angle to zero");
        telemetry.addLine("LED: white=manual, red=aligning, green=locked");
        turretSubsystem.Alliance testAlliance = turretSubsystem.Alliance.BLUE;
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) testAlliance = turretSubsystem.Alliance.RED;
            if (gamepad1.b) testAlliance = turretSubsystem.Alliance.BLUE;
            turret.setAlliance(testAlliance);
            telemetry.addData("TEST ALLIANCE", testAlliance);
            telemetry.addLine("Before START: X = RED, B = BLUE");
            telemetry.update();
            idle();
        }
        if (isStopRequested()) {
            turret.stop();
            shooter.stop();
            robot.intake.setPower(0);
            robot.transfer.setPower(0);
            rec.close();
            return;
        }

        rec.event("OPMODE_START tune=" + turretSubsystem.TUNE_VERSION
                + " alliance=" + testAlliance);
        loopTimer.reset();
        try {
            while (opModeIsActive()) {
                robot.beginLoop();
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
                    setShooterAndFeeders(robot, shooter, false);
                } else if (aPressed && !lastShooterButton) {
                    setShooterAndFeeders(robot, shooter, !shooter.isEnabled());
                }
                lastShooterButton = aPressed;

                // Set target RPM for distance if turret has distance, otherwise default
                if (turret.hasDistance()) {
                    shooter.setTargetForDistance(turret.getShooterDistanceIn());
                } else {
                    shooter.setTargetRpm(shooterSubsystem.COMPETITION_RPM);
                }
                shooter.update(dtSeconds);

                // D-pad up toggles odometry fusion so a run can compare it against vision-only aiming.
                if (gamepad1.dpad_up && !lastFusionToggle) {
                    turretSubsystem.ODOMETRY_FUSION_ENABLED = !turretSubsystem.ODOMETRY_FUSION_ENABLED;
                    rec.event("FUSION_TOGGLE enabled=" + turretSubsystem.ODOMETRY_FUSION_ENABLED);
                }
                lastFusionToggle = gamepad1.dpad_up;

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

                // 4. Flight recorder row + telemetry
                turret.logTo(rec);
                rec.put("op_dt_ms", dtSeconds * 1000.0);
                rec.put("in_forward", forward);
                rec.put("in_strafe", strafe);
                rec.put("in_turn", turn);
                rec.put("in_autoaim", autoAim);
                rec.put("in_dpad_manual", manualTurret);
                shooter.logTo(rec);
                rec.endRow();

                telemetry.addData("FUSION (D-pad up)", turretSubsystem.ODOMETRY_FUSION_ENABLED ? "ON" : "OFF (vision only)");
                telemetry.addData("Log", rec.getStatus());
                telemetry.addData("Ready after stop (ms)", "%.0f", turret.getReadyAfterStopMs());
                telemetry.addData("Loop", "%.1f ms (turret %.0f us)", dtSeconds * 1000.0,
                        turret.getUpdateMicros());
                telemetry.addData("Drive", "Fwd=%.2f, Strafe=%.2f, Turn=%.2f", forward, strafe, turn);
                telemetry.addData("Turret Mode", autoAim ? "AUTO-AIM" : "MANUAL");
                telemetry.addData("Turret Tune", turretSubsystem.TUNE_VERSION);
                telemetry.addData("Alliance / CELL", "%s / %s",
                        turret.getAlliance(), turret.getSelectedClusterName());
                telemetry.addData("Target Visible", turret.hasTarget()
                        ? "YES (" + turret.getVisibleTagCount() + " tags: "
                          + turret.getVisibleTagIds() + ")"
                        : turret.isOdometryTrackingActive()
                          ? "ODOMETRY (vision lost)" : "NO");
                telemetry.addData("Hive Field Memory",
                        !turret.hasFieldTargetMemory() ? "EMPTY"
                                : turret.isOdometryPoseValid()
                                        ? "SAVED / TRACKING"
                                        : "SAVED / ODOMETRY PAUSED");
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
                telemetry.addData("PID Gains", "P=%.4f I=%.4f D=%.4f kV=%.4f kS=%.3f",
                        turretSubsystem.POSITION_KP,
                        turretSubsystem.POSITION_KI,
                        turretSubsystem.VELOCITY_KD,
                        turretSubsystem.VELOCITY_KV,
                        turretSubsystem.STATIC_FRICTION_POWER);
                telemetry.addData("Turret Vision", "tx=%+.2f deg, err=%+.2f deg",
                        turret.getFilteredTxDeg(), turret.getLastVisionErrorDeg());
                telemetry.addData("Shooter", "Enabled=%s, Actual RPM=%.0f", shooter.isEnabled() ? "ON" : "OFF", shooter.getActualRpm());
                telemetry.update();
            }
        } catch (Throwable t) {
            rec.exception("turret_biobuzzTestTeleOp loop", t);
            throw t;
        } finally {
            rec.event("OPMODE_END");
            rec.close();
            robot.lf.setPower(0);
            robot.lb.setPower(0);
            robot.rf.setPower(0);
            robot.rb.setPower(0);
            turret.stop();
            shooter.stop();
            robot.intake.setPower(0);
            robot.transfer.setPower(0);
        }
    }

    private void setShooterAndFeeders(
            Hardware robot, shooterSubsystem shooter, boolean enabled) {
        shooter.setShooterEnabled(enabled);
        robot.intake.setPower(enabled ? INTAKE_POWER : 0);
        robot.transfer.setPower(enabled ? TRANSFER_POWER : 0);
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < 0.05 ? 0 : value;
    }
}
