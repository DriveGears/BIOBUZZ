package org.firstinspires.ftc.teamcode.opmodes.competitionTeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.FlightRecorder;
import org.firstinspires.ftc.teamcode.subsystems.shooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.turretSubsystem;

import java.util.Map;

/** Shared implementation for the alliance-specific competition TeleOps. */
public abstract class biobuzzTeleop extends LinearOpMode {

    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_POWER = 1.0;
    private static final double TRANSFER_TICKS_PER_REVOLUTION = 28.0;
    private static final double MANUAL_TURRET_POWER = 0.40;
    private static final double DRIVE_STICK_DEADBAND = 0.05;
    private static final double DASHBOARD_PERIOD_SECONDS = 0.050;
    private static final double OUTTAKE_CURRENT_SAMPLE_SECONDS = 0.100;

    private FlightRecorder rec;
    private turretSubsystem turret;
    private shooterSubsystem shooter;
    private ElapsedTime loopTimer;
    private ElapsedTime dashboardTimer;
    private ElapsedTime outtakeCurrentTimer;
    private FtcDashboard dashboard;
    private double outtakeCurrentAmps = Double.NaN;

    private boolean intakeToggle = false;
    private boolean outtakeToggle = false;
    private boolean lastRightTrigger = false;
    private boolean lastLeftTrigger = false;
    private boolean lastLeftBumper = false;
    private boolean autoAimEnabled = true;

    protected abstract turretSubsystem.Alliance competitionAlliance();

    @Override
    public void runOpMode() {
        Hardware.resetInstance();
        Hardware robot = Hardware.getInstance(hardwareMap);
        // Test OpModes can toggle this static setting. Competition TeleOp always
        // starts with field-relative hive memory enabled and cannot disable it accidentally.
        turretSubsystem.ODOMETRY_FUSION_ENABLED = true;

        // The Pedro Follower was only used for a pose readout and cost a second Pinpoint I2C read every
        // loop (the turret already reads Pinpoint). Pose now comes from the Pinpoint via Hardware.
        turret = new turretSubsystem(robot);
        shooter = new shooterSubsystem(robot);
        loopTimer = new ElapsedTime();
        dashboardTimer = new ElapsedTime();
        outtakeCurrentTimer = new ElapsedTime();
        dashboard = FtcDashboard.getInstance();
        robot.intake.setPower(0);
        robot.transfer.setPower(0);

        rec = FlightRecorder.start("teleop");
        Map<String, String> cfg = turretSubsystem.configSnapshot();
        cfg.put("opmode", getClass().getSimpleName());
        cfg.put("tune_version", turretSubsystem.TUNE_VERSION);
        rec.writeConfig(cfg);
        rec.event("--- ODOMETRY INIT ---\n" + robot.getOdometryReport());
        turret.setRecorder(rec);
        robot.enableManualBulkReads();     // one hub read per loop; beginLoop() below refreshes it
        telemetry.setMsTransmissionInterval(100);

        turret.setAlliance(competitionAlliance());
        turret.setAutoAimEnabled(true);
        shooter.setTargetRpm(shooterSubsystem.COMPETITION_RPM);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Robot Initialized with Turret & Robot-Centric Drive");
            telemetry.addData("Fixed Alliance", turret.getAlliance());
            telemetry.addLine("Auto-aim will start ENABLED");
            addControlsTelemetry();
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

        rec.event("OPMODE_START tune=" + turretSubsystem.TUNE_VERSION + " alliance=" + competitionAlliance());
        loopTimer.reset();
        dashboardTimer.reset();
        outtakeCurrentTimer.reset();
        lastRightTrigger = gamepad1.right_trigger > 0.5;
        lastLeftTrigger = gamepad1.left_trigger > 0.5;
        lastLeftBumper = gamepad1.left_bumper;

        try {
        while (opModeIsActive()) {
            robot.beginLoop();
            double dtSeconds = loopTimer.seconds();
            loopTimer.reset();

            double forward = -applyDeadband(gamepad1.left_stick_y);
            double lateral = applyDeadband(gamepad1.left_stick_x);
            double turn = applyDeadband(gamepad1.right_stick_x);

            boolean currentRightTrigger = gamepad1.right_trigger > 0.5;
            if (currentRightTrigger && !lastRightTrigger) {
                intakeToggle = !intakeToggle;
            }
            lastRightTrigger = currentRightTrigger;

            boolean currentLeftTrigger = gamepad1.left_trigger > 0.5;
            if (currentLeftTrigger && !lastLeftTrigger) {
                outtakeToggle = !outtakeToggle;
            }
            lastLeftTrigger = currentLeftTrigger;

            boolean spit = gamepad1.right_bumper;

            if (spit) {
                robot.intake.setPower(-INTAKE_POWER);
            } else {
                robot.intake.setPower(intakeToggle ? INTAKE_POWER : 0.0);
            }

            if (turret.hasDistance()) {
                shooter.setTargetForDistance(turret.getShooterDistanceIn());
            } else {
                shooter.setTargetRpm(shooterSubsystem.COMPETITION_RPM);
            }
            shooter.setShooterEnabled(outtakeToggle);
            shooter.update(dtSeconds);

            boolean transferRequested = gamepad1.a;
            boolean transferReverse = gamepad1.x;

            // Auto-aim starts enabled. Tap LB to switch to/from manual aiming.
            boolean currentLeftBumper = gamepad1.left_bumper;
            if (currentLeftBumper && !lastLeftBumper) {
                autoAimEnabled = !autoAimEnabled;
            }
            lastLeftBumper = currentLeftBumper;
            turret.setAutoAimEnabled(autoAimEnabled);

            double manualTurretCommand = (gamepad1.dpad_right ? 1.0 : 0.0)
                    - (gamepad1.dpad_left ? 1.0 : 0.0);
            turret.setManualPower(manualTurretCommand * MANUAL_TURRET_POWER);
            turret.update(dtSeconds);

            // Keep readiness as telemetry for the driver/log, but A always owns
            // forward transfer directly. The driver decides when to release a shot.
            boolean shotReady = shooter.isAtSpeed()
                    && (!autoAimEnabled || turret.isAimReady());
            boolean transferFeeding = transferRequested;
            if (transferFeeding) {
                robot.transfer.setPower(TRANSFER_POWER);
            } else if (transferReverse) {
                robot.transfer.setPower(-TRANSFER_POWER);
            } else {
                robot.transfer.setPower(0);
            }

            if (outtakeCurrentTimer.seconds() >= OUTTAKE_CURRENT_SAMPLE_SECONDS) {
                outtakeCurrentTimer.reset();
                try {
                    outtakeCurrentAmps = robot.readOuttakeCurrentAmps();
                } catch (Throwable ignored) { }
            }

            double transferCommand = transferFeeding ? TRANSFER_POWER
                    : transferReverse ? -TRANSFER_POWER : 0.0;
            double transferRpm = robot.transfer.getVelocity() * 60.0
                    / TRANSFER_TICKS_PER_REVOLUTION;

            // Auto-aim moves only the turret. Chassis rotation is always the
            // driver's right-stick command.
            double driveTurn = turn;

            double denominator = Math.max(
                    Math.abs(forward) + Math.abs(lateral) + Math.abs(driveTurn), 1.0);
            robot.lf.setPower((forward + lateral + driveTurn) / denominator);
            robot.lb.setPower((forward - lateral + driveTurn) / denominator);
            robot.rf.setPower((forward - lateral - driveTurn) / denominator);
            robot.rb.setPower((forward + lateral - driveTurn) / denominator);

            // ---- flight recorder row ----
            turret.logTo(rec);
            rec.put("op_dt_ms", dtSeconds * 1000.0);
            rec.put("in_forward", forward);
            rec.put("in_lateral", lateral);
            rec.put("in_turn", turn);
            rec.put("in_autoaim", autoAimEnabled);
            rec.put("in_dpad_manual", manualTurretCommand);
            rec.put("in_intake_on", intakeToggle);
            rec.put("in_outtake_on", outtakeToggle);
            rec.put("in_transfer_requested", transferRequested);
            rec.put("shot_ready", shotReady);
            rec.put("in_transfer_command", transferCommand);
            rec.put("transfer_actual_rpm", transferRpm);
            rec.put("shoot_current_a", outtakeCurrentAmps);
            shooter.logTo(rec);
            rec.endRow();

            sendDashboardTelemetry(
                    dtSeconds, transferRequested, shotReady,
                    transferCommand, transferRpm);

            telemetry.addData("Turret Mode", autoAimEnabled ? "AUTO-AIM" : "MANUAL");
            telemetry.addData("Target Visible", turret.hasTarget()
                    ? "YES (ID: " + turret.getSelectedTagId() + ")"
                    : turret.isOdometryTrackingActive()
                            ? "ODOMETRY MEMORY" : "NO");
            telemetry.addData("Hive Field Memory",
                    !turret.hasFieldTargetMemory() ? "EMPTY"
                            : turret.isOdometryPoseValid()
                                    ? "SAVED / TRACKING"
                                    : "SAVED / ODOMETRY PAUSED");
            telemetry.addData("Turret Ticks", "current=%d target=%.1f limits=[%d, %d]",
                    turret.getCurrentTicks(), turret.getTargetTicks(),
                    turretSubsystem.MIN_TURRET_TICKS,
                    turretSubsystem.MAX_TURRET_TICKS);
            telemetry.addData("Turret Raw / Zero", "%d / %d",
                    robot.getTurretRawEncoderTicks(),
                    robot.getTurretEncoderZeroTicks());
            telemetry.addData("Allowed visible IDs", turret.getVisibleTagIds());
            telemetry.addData("Alliance / CELL", "%s / %s",
                    turret.getAlliance(), turret.getSelectedClusterName());
            telemetry.addData("CELL Roll", Double.isFinite(turret.getSelectedClusterRollDeg())
                    ? String.format("%.1f deg (UP)", turret.getSelectedClusterRollDeg())
                    : "no scorable CELL");
            telemetry.addData("Prediction Lead", "%.2f deg", turret.getPanLeadDeg());
            double shooterDistanceIn = turret.getShooterDistanceIn();
            telemetry.addData("Shooter Range",
                    turret.hasDistance() && Double.isFinite(shooterDistanceIn)
                            && shooterDistanceIn > 0
                            ? String.format(java.util.Locale.US, "%.1f in", shooterDistanceIn)
                            : "NO VALID RANGE (RPM fallback active)");
            telemetry.addData("Range Source",
                    turret.hasDistance() && Double.isFinite(shooterDistanceIn)
                            ? turret.hasTarget() ? "LIVE TAG" : "HELD ODOMETRY"
                            : turret.hasTarget()
                                    ? "TAG VISIBLE; CAMERA GEOMETRY INVALID"
                                    : "NO ALLOWED TAG");
            telemetry.addData("Chassis Aim Assist", "DISABLED");
            telemetry.addData("Aim Ready?", turret.isAimReady() ? "READY" : "ALIGNING");
            telemetry.addData("Intake Toggle", intakeToggle ? "ON" : "OFF");
            telemetry.addData("Outtake Toggle", outtakeToggle ? "ON" : "OFF");
            telemetry.addData("Shooter RPM", "target=%.0f actual=%.0f %s",
                    shooter.getTargetRpm(), shooter.getActualRpm(),
                    !outtakeToggle ? "OFF"
                            : shooter.isAtSpeed() ? "READY" : "SPINNING UP");
            telemetry.addData("Transfer", transferFeeding ? "FEEDING (A)"
                    : transferReverse ? "REVERSING (X)" : "OFF");
            telemetry.addData("Odo pose", "x=%.1f y=%.1f hdg=%.1f",
                    robot.odo.getPosX(DistanceUnit.INCH),
                    robot.odo.getPosY(DistanceUnit.INCH),
                    Math.toDegrees(robot.odo.getHeading(
                            AngleUnit.RADIANS)));
            telemetry.addData("FUSION", "ON (competition locked)");
            telemetry.addData("Ready after stop (ms)", "%.0f", turret.getReadyAfterStopMs());
            telemetry.addData("Log", rec.getStatus());
            addControlsTelemetry();

            telemetry.update();
        }
        } catch (Throwable t) {
            rec.exception("biobuzzTeleop loop", t);
            throw t;
        } finally {
            robot.lf.setPower(0);
            robot.lb.setPower(0);
            robot.rf.setPower(0);
            robot.rb.setPower(0);
            turret.stop();
            shooter.stop();
            robot.intake.setPower(0);
            robot.transfer.setPower(0);
            rec.event("OPMODE_END");
            rec.close();
        }
    }

    /** Numeric-only packet: each stable key can be selected directly in Dashboard graphs. */
    private void sendDashboardTelemetry(
            double dtSeconds, boolean transferRequested, boolean shotReady,
            double transferCommand, double transferRpm) {
        if (dashboardTimer.seconds() < DASHBOARD_PERIOD_SECONDS) return;
        dashboardTimer.reset();

        double actualRpm = shooter.getActualRpm();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("shooter.target_rpm", shooter.getTargetRpm());
        packet.put("shooter.commanded_rpm", shooter.getCommandedRpm());
        packet.put("shooter.actual_rpm", actualRpm);
        packet.put("shooter.error_rpm", shooter.getTargetRpm() - actualRpm);
        packet.put("shooter.power", shooter.getAppliedPower());
        packet.put("shooter.p_term", shooter.getPTerm());
        packet.put("shooter.i_term", shooter.getITerm());
        packet.put("shooter.d_term", shooter.getDTerm());
        packet.put("shooter.f_term", shooter.getFTerm());
        packet.put("shooter.ready", shooter.isAtSpeed() ? 1.0 : 0.0);
        putIfFinite(packet, "shooter.distance_bucket_in", shooter.getDistanceBucketIn());
        putIfFinite(packet, "shooter.current_a", outtakeCurrentAmps);

        packet.put("transfer.command", transferCommand);
        packet.put("transfer.actual_rpm", transferRpm);
        packet.put("transfer.requested", transferRequested ? 1.0 : 0.0);
        packet.put("transfer.shot_ready", shotReady ? 1.0 : 0.0);

        packet.put("turret.current_ticks", turret.getCurrentTicks());
        packet.put("turret.raw_encoder_ticks", turret.getRawEncoderTicks());
        packet.put("turret.zero_offset_ticks", turret.getEncoderZeroTicks());
        packet.put("turret.target_ticks", turret.getTargetTicks());
        packet.put("turret.error_ticks", turret.getPositionErrorTicks());
        packet.put("turret.velocity_tps", turret.getFilteredMotorVelocityTicksPerSecond());
        packet.put("turret.power", turret.getAppliedPower());
        packet.put("turret.tx_deg", turret.getFilteredTxDeg());
        packet.put("turret.aim_ready", turret.isAimReady() ? 1.0 : 0.0);
        packet.put("turret.target_visible", turret.hasTarget() ? 1.0 : 0.0);
        packet.put("turret.field_memory_valid", turret.hasFieldTargetMemory() ? 1.0 : 0.0);
        packet.put("turret.odometry_pose_valid", turret.isOdometryPoseValid() ? 1.0 : 0.0);
        packet.put("turret.range_valid", turret.hasDistance() ? 1.0 : 0.0);
        packet.put("turret.selected_tag_id", turret.getSelectedTagId());
        packet.put("turret.cluster_first_id", turret.getSelectedClusterFirstId());
        putIfFinite(packet, "turret.distance_in", turret.getDistanceIn());
        putIfFinite(packet, "turret.impact_distance_in", turret.getImpactDistanceIn());
        putIfFinite(packet, "turret.fused_distance_in", turret.getFusedDistanceIn());
        putIfFinite(packet, "turret.shooter_distance_in", turret.getShooterDistanceIn());
        packet.put("turret.fused_distance_valid", turret.hasFusedDistance() ? 1.0 : 0.0);
        packet.put("turret.range_rate_in_s", turret.getRangeRateInPerSec());
        packet.put("turret.robot_speed_in_s", turret.getRobotSpeedInPerSec());
        packet.put("turret.geometry_valid", turret.isClusterGeometryValid() ? 1.0 : 0.0);
        putIfFinite(packet, "battery.volts", turret.getBatteryVolts());
        packet.put("loop.ms", dtSeconds * 1000.0);

        dashboard.sendTelemetryPacket(packet);
    }

    private void putIfFinite(TelemetryPacket packet, String key, double value) {
        if (Double.isFinite(value)) packet.put(key, value);
    }

    private static double applyDeadband(double value) {
        return Math.abs(value) < DRIVE_STICK_DEADBAND ? 0.0 : value;
    }

    private void addControlsTelemetry() {
        telemetry.addLine();
        telemetry.addLine("=== GAMEPAD 1 CONTROLS ===");
        telemetry.addData("Alliance", competitionAlliance());
        telemetry.addLine("Left Stick: Drive/strafe | Right Stick X: Turn");
        telemetry.addLine("RT (tap): Toggle intake | RB (hold): Reverse intake");
        telemetry.addLine("LT (tap): Toggle flywheel | A (hold): Transfer forward | X (hold): Reverse transfer");
        telemetry.addLine("LB (tap): Toggle auto/manual (starts AUTO)");
        telemetry.addLine("D-pad L/R: Move turret while in manual mode");
        telemetry.addLine("Auto-aim moves turret only; right stick owns chassis turn");
    }
}
