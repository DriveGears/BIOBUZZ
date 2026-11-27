package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.OdometryConfig;
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
    /** Official cluster dimensions are intentionally excluded; these are robot calibrations. */
    private enum AimCalibration {
        FINAL_AIM_OFFSET_DEG("AIM_OFFSET_DEG", "deg", 0.10,
                "Final left/right trim applied to every shot"),
        AIM_ANGLE_SCALE("AIM_ANGLE_SCALE", "x", 0.005,
                "Corrects turret angle scale equally on both sides"),
        CAMERA_LEFT_OF_SHOOTER_IN("CAMERA_LEFT_OF_SHOOTER_IN", "in", 0.10,
                "Measured camera position left of the shooter axis"),
        CAMERA_HEIGHT_IN("CAMERA_HEIGHT_IN", "in", 0.10,
                "Measured Limelight lens height from the floor"),
        CAMERA_UP_ANGLE_DEG("CAMERA_UP_ANGLE_DEG", "deg", 0.10,
                "Measured Limelight upward mounting pitch"),
        DEFAULT_TAG_HEIGHT_IN("DEFAULT_TAG_HEIGHT_IN", "in", 0.10,
                "Measured AprilTag center height from the floor"),
        BANK_AIM_CENTER_OFFSET_IN("BANK_AIM_CENTER_OFFSET_IN", "in", 0.25,
                "Left/right target distance even when shooter is centered"),
        BANK_AIM_MAX_OFFSET_IN("BANK_AIM_MAX_OFFSET_IN", "in", 0.25,
                "Maximum aim point distance left/right of hive center"),
        BANK_AIM_FULL_SIDE_IN("BANK_AIM_FULL_SIDE_IN", "in", 0.25,
                "Shooter side distance where maximum bank offset is reached"),
        BANK_AIM_SWITCH_HYSTERESIS_IN("BANK_AIM_SWITCH_HYSTERESIS_IN", "in", 0.25,
                "Must cross this far past center before changing aim side"),
        BANK_AIM_SWITCH_HOLD_MS("BANK_AIM_SWITCH_HOLD_MS", "ms", 50.0,
                "Minimum time between left/right aim-side changes"),
        TURRET_PIVOT_FORWARD_IN("TURRET_PIVOT_FORWARD_IN", "in", 0.10,
                "Turret pivot position forward of the odometry origin"),
        TURRET_PIVOT_LEFT_IN("TURRET_PIVOT_LEFT_IN", "in", 0.10,
                "Turret pivot position left of the odometry origin");

        final String label;
        final String unit;
        final double step;
        final String help;

        AimCalibration(String label, String unit, double step, String help) {
            this.label = label;
            this.unit = unit;
            this.step = step;
            this.help = help;
        }
    }

    private boolean lastShooterButton = false;
    private boolean lastTurretZeroButton = false;
    private boolean lastFusionToggle = false;
    private boolean lastCalibrationDecrease = false;
    private boolean lastCalibrationIncrease = false;
    private boolean lastCalibrationReset = false;
    private boolean lastCalibrationNext = false;
    private boolean lastCalibrationPrevious = false;

    @Override
    public void runOpMode() {
        Hardware.resetInstance();
        // Every test starts with the competition fusion path enabled. D-pad up
        // can still toggle it during a run for a vision-only comparison.
        turretSubsystem.ODOMETRY_FUSION_ENABLED = true;
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
        addControlsTelemetry();
        telemetry.addLine("LED: blue=manual, red=aligning, yellow=odometry lock, green=live vision lock");
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
        AimCalibration selectedCalibration = AimCalibration.FINAL_AIM_OFFSET_DEG;
        double[] calibrationStartValues = snapshotCalibrationValues(turret);
        lastCalibrationDecrease = gamepad1.left_bumper;
        lastCalibrationIncrease = gamepad1.right_bumper;
        lastCalibrationReset = gamepad1.y;
        lastCalibrationNext = gamepad1.x;
        lastCalibrationPrevious = gamepad1.dpad_down;
        lastTurretZeroButton = gamepad1.start;
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

                // Select and adjust every robot-specific aim geometry value. Values
                // are restored in finally so a test cannot leak into competition.
                boolean calibrationDecrease = gamepad1.left_bumper;
                boolean calibrationIncrease = gamepad1.right_bumper;
                boolean calibrationReset = gamepad1.y;
                boolean calibrationNext = gamepad1.x;
                boolean calibrationPrevious = gamepad1.dpad_down;
                AimCalibration[] calibrations = AimCalibration.values();
                if (calibrationNext && !lastCalibrationNext) {
                    selectedCalibration = calibrations[
                            (selectedCalibration.ordinal() + 1) % calibrations.length];
                } else if (calibrationPrevious && !lastCalibrationPrevious) {
                    selectedCalibration = calibrations[
                            (selectedCalibration.ordinal() + calibrations.length - 1)
                                    % calibrations.length];
                } else if (calibrationReset && !lastCalibrationReset) {
                    setCalibrationValue(turret, selectedCalibration,
                            calibrationStartValues[selectedCalibration.ordinal()]);
                    rec.event("CALIBRATION_RESET name=" + selectedCalibration.label
                            + " value=" + getCalibrationValue(turret, selectedCalibration));
                } else if (calibrationDecrease && !lastCalibrationDecrease) {
                    adjustCalibration(turret, selectedCalibration, -selectedCalibration.step);
                    rec.event("CALIBRATION_DECREASE name=" + selectedCalibration.label
                            + " value=" + getCalibrationValue(turret, selectedCalibration));
                } else if (calibrationIncrease && !lastCalibrationIncrease) {
                    adjustCalibration(turret, selectedCalibration, selectedCalibration.step);
                    rec.event("CALIBRATION_INCREASE name=" + selectedCalibration.label
                            + " value=" + getCalibrationValue(turret, selectedCalibration));
                }
                lastCalibrationDecrease = calibrationDecrease;
                lastCalibrationIncrease = calibrationIncrease;
                lastCalibrationReset = calibrationReset;
                lastCalibrationNext = calibrationNext;
                lastCalibrationPrevious = calibrationPrevious;

                // 3. Turret Auto-Aim & Manual Control
                boolean turretZeroPressed = gamepad1.start;
                if (turretZeroPressed && !lastTurretZeroButton) {
                    turret.zeroAtCurrentPosition();
                }
                lastTurretZeroButton = turretZeroPressed;

                // Do not allow auto aim in the same loop as a zero operation.
                boolean autoAim = gamepad1.right_trigger > 0.20 && !turretZeroPressed;
                boolean turretPaused = gamepad1.back;
                turret.setMovementPaused(turretPaused);
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
                rec.put("in_turret_paused", turretPaused);
                rec.put("in_dpad_manual", manualTurret);
                rec.put("tune_calibration_index", selectedCalibration.ordinal());
                rec.put("tune_calibration_value",
                        getCalibrationValue(turret, selectedCalibration));
                rec.put("tune_calibration_start_value",
                        calibrationStartValues[selectedCalibration.ordinal()]);
                shooter.logTo(rec);
                rec.endRow();

                addControlsTelemetry();
                telemetry.addData("FUSION (D-pad up)", turretSubsystem.ODOMETRY_FUSION_ENABLED ? "ON" : "OFF (vision only)");
                telemetry.addData("Log", rec.getStatus());
                telemetry.addData("Ready after stop (ms)", "%.0f", turret.getReadyAfterStopMs());
                telemetry.addData("Loop", "%.1f ms (turret %.0f us)", dtSeconds * 1000.0,
                        turret.getUpdateMicros());
                telemetry.addData("Drive", "Fwd=%.2f, Strafe=%.2f, Turn=%.2f", forward, strafe, turn);
                telemetry.addData("Turret Mode", autoAim ? "AUTO-AIM" : "MANUAL");
                telemetry.addData("Turret Pause", turretPaused
                        ? "HELD - tracking continues, motor stopped" : "released");
                telemetry.addData("Aim Source", "%s (%s)",
                        turret.getTrackingState(), turret.getTrackingStateReason());
                telemetry.addData("LED Command", "position=%.3f, pulse=%.0f us",
                        turret.getIndicatorPosition(), turret.getIndicatorPulseUs());
                telemetry.addData("Turret Tune", turretSubsystem.TUNE_VERSION);
                telemetry.addData("Selected Calibration", "%d/%d %s",
                        selectedCalibration.ordinal() + 1,
                        AimCalibration.values().length,
                        selectedCalibration.label);
                telemetry.addData("Meaning", selectedCalibration.help);
                telemetry.addData("Live / Start / Step", "%+.3f / %+.3f / %.3f %s",
                        getCalibrationValue(turret, selectedCalibration),
                        calibrationStartValues[selectedCalibration.ordinal()],
                        selectedCalibration.step, selectedCalibration.unit);
                telemetry.addData("Save After Test", "copy %s = %+.3f",
                        selectedCalibration.label,
                        getCalibrationValue(turret, selectedCalibration));
                telemetry.addLine("Official CELL/tag geometry is fixed and is not tuned here.");
                telemetry.addData("Alliance / CELL", "%s / %s",
                        turret.getAlliance(), turret.getSelectedClusterName());
                telemetry.addData("Side Aim", "camera=%+.1f shooter=%+.1f, offset=%+.1f in (%+.2f deg)",
                        turret.getCameraSideOfClusterIn(), turret.getShooterSideOfClusterIn(),
                        turret.getBankAimOffsetIn(), turret.getBankAimCorrectionDeg());
                telemetry.addData("Bank Target", "%s side of hive",
                        turret.getBankAimSide());
                telemetry.addLine("Bank rule: shooter LEFT -> aim RIGHT; shooter RIGHT -> aim LEFT");
                double bankMaxIn = Math.max(0,
                        Math.abs(turretSubsystem.BANK_AIM_MAX_OFFSET_IN));
                double bankCenterIn = Math.min(bankMaxIn, Math.max(0,
                        Math.abs(turretSubsystem.BANK_AIM_CENTER_OFFSET_IN)));
                double bankFullSideIn = Math.max(0.1,
                        Math.abs(turretSubsystem.BANK_AIM_FULL_SIDE_IN));
                double bankHalfIn = bankCenterIn
                        + (bankMaxIn - bankCenterIn) * 0.5;
                telemetry.addData("Bank CENTER_OFFSET",
                        "%.1f in = target shift while shooter is centered", bankCenterIn);
                telemetry.addData("Bank MAX_OFFSET",
                        "%.1f in = largest target shift from hive center", bankMaxIn);
                telemetry.addData("Bank FULL_SIDE",
                        "%.1f in = shooter side distance that reaches full %.1f in shift",
                        bankFullSideIn, bankMaxIn);
                telemetry.addData("Bank Example",
                        "RIGHT %.1f in -> aim LEFT %.1f in (LEFT side mirrors this)",
                        bankFullSideIn / 2.0, bankHalfIn);
                telemetry.addData("Bank Switch Guards",
                        "cross %.1f in past center | hold %.0f ms | %.0f ms remaining",
                        turretSubsystem.BANK_AIM_SWITCH_HYSTERESIS_IN,
                        turretSubsystem.BANK_AIM_SWITCH_HOLD_MS,
                        turret.getBankAimSwitchRemainingMs());
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
                telemetry.addData("Turret Encoder", "relative=%d = raw %d - saved zero %d",
                        turret.getCurrentTicks(), robot.getTurretRawEncoderTicks(),
                        robot.getTurretEncoderZeroTicks());
                telemetry.addData("Turret Safety", turret.hasHardLimitFault()
                        ? "HARD LIMIT: auto blocked; use manual only toward center, then START"
                        : "relative limits OK");
                telemetry.addData("Turret Limit",
                        turret.getCurrentTicks() <= turretSubsystem.MIN_TURRET_TICKS
                                ? "LEFT SOFT LIMIT"
                                : turret.getCurrentTicks() >= turretSubsystem.MAX_TURRET_TICKS
                                  ? "RIGHT SOFT LIMIT" : "clear");
                telemetry.addData("Turret PD", "err=%+.1f tk, vel=%+.1f tk/s, pwr=%+.3f",
                        turret.getPositionErrorTicks(),
                        turret.getFilteredMotorVelocityTicksPerSecond(),
                        turret.getAppliedPower());
                telemetry.addData("Controller Gains", "P=%.4f D=%.4f kS=%.3f",
                        turretSubsystem.POSITION_KP,
                        turretSubsystem.VELOCITY_KD,
                        turretSubsystem.STATIC_FRICTION_POWER);
                telemetry.addData("Turret Vision", "tx=%+.2f deg, err=%+.2f deg",
                        turret.getFilteredTxDeg(), turret.getLastVisionErrorDeg());
                telemetry.addData("Range", "used=%.1f raw=%.1f filtered=%.1f fused=%.1f",
                        turret.getShooterDistanceIn(), turret.getRawDistanceIn(),
                        turret.getDistanceIn(), turret.getFusedDistanceIn());
                telemetry.addData("Range Protection", "shotFreeze=%s disagreement=%+.1f in",
                        turret.isDistanceFrozenAfterShot(), turret.getRangeDisagreementIn());
                telemetry.addData("Shooter", "Enabled=%s, Actual RPM=%.0f", shooter.isEnabled() ? "ON" : "OFF", shooter.getActualRpm());
                telemetry.update();
            }
        } catch (Throwable t) {
            rec.exception("turret_biobuzzTestTeleOp loop", t);
            throw t;
        } finally {
            rec.event("--- CALIBRATION RESULTS (copy into source) ---\n"
                    + calibrationSummary(turret));
            restoreCalibrationValues(turret, calibrationStartValues);
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

    private void addControlsTelemetry() {
        telemetry.addLine("=== TURRET TEST CONTROLS ===");
        telemetry.addLine("Drive: left stick move | right stick X turn");
        telemetry.addLine("Aim: hold RT auto | BACK hold pause | D-pad L/R manual | START zero");
        telemetry.addLine("Shooter/feeders: A toggle ON | B OFF");
        telemetry.addLine("Tune select: X next | D-pad DOWN previous");
        telemetry.addLine("Tune value: LB decrease | RB increase | Y reset selected");
        telemetry.addLine("Compare aiming: D-pad UP toggles odometry fusion");
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < 0.05 ? 0 : value;
    }

    private static double[] snapshotCalibrationValues(turretSubsystem turret) {
        AimCalibration[] calibrations = AimCalibration.values();
        double[] values = new double[calibrations.length];
        for (AimCalibration calibration : calibrations) {
            values[calibration.ordinal()] = getCalibrationValue(turret, calibration);
        }
        return values;
    }

    private static void restoreCalibrationValues(
            turretSubsystem turret, double[] values) {
        for (AimCalibration calibration : AimCalibration.values()) {
            setCalibrationValue(turret, calibration, values[calibration.ordinal()]);
        }
    }

    private static String calibrationSummary(turretSubsystem turret) {
        StringBuilder summary = new StringBuilder();
        for (AimCalibration calibration : AimCalibration.values()) {
            summary.append(calibration.label).append('=')
                    .append(String.format(java.util.Locale.US, "%.4f",
                            getCalibrationValue(turret, calibration)))
                    .append(' ').append(calibration.unit).append('\n');
        }
        return summary.toString();
    }

    private static void adjustCalibration(
            turretSubsystem turret, AimCalibration calibration, double delta) {
        setCalibrationValue(turret, calibration,
                getCalibrationValue(turret, calibration) + delta);
    }

    private static double getCalibrationValue(
            turretSubsystem turret, AimCalibration calibration) {
        switch (calibration) {
            case FINAL_AIM_OFFSET_DEG: return turret.getTotalAimOffsetDeg();
            case AIM_ANGLE_SCALE: return turretSubsystem.AIM_ANGLE_SCALE;
            case CAMERA_LEFT_OF_SHOOTER_IN: return turretSubsystem.CAMERA_LEFT_OF_SHOOTER_IN;
            case CAMERA_HEIGHT_IN: return turretSubsystem.CAMERA_HEIGHT_IN;
            case CAMERA_UP_ANGLE_DEG: return turretSubsystem.CAMERA_UP_ANGLE_DEG;
            case DEFAULT_TAG_HEIGHT_IN: return turretSubsystem.DEFAULT_TAG_HEIGHT_IN;
            case BANK_AIM_CENTER_OFFSET_IN: return turretSubsystem.BANK_AIM_CENTER_OFFSET_IN;
            case BANK_AIM_MAX_OFFSET_IN: return turretSubsystem.BANK_AIM_MAX_OFFSET_IN;
            case BANK_AIM_FULL_SIDE_IN: return turretSubsystem.BANK_AIM_FULL_SIDE_IN;
            case BANK_AIM_SWITCH_HYSTERESIS_IN: return turretSubsystem.BANK_AIM_SWITCH_HYSTERESIS_IN;
            case BANK_AIM_SWITCH_HOLD_MS: return turretSubsystem.BANK_AIM_SWITCH_HOLD_MS;
            case TURRET_PIVOT_FORWARD_IN: return OdometryConfig.TURRET_PIVOT_FORWARD_IN;
            case TURRET_PIVOT_LEFT_IN: return OdometryConfig.TURRET_PIVOT_LEFT_IN;
            default: throw new IllegalArgumentException("Unknown calibration " + calibration);
        }
    }

    private static void setCalibrationValue(
            turretSubsystem turret, AimCalibration calibration, double value) {
        switch (calibration) {
            case FINAL_AIM_OFFSET_DEG:
                turret.setRuntimeAimTrimDeg(clamp(value - turretSubsystem.AIM_OFFSET_DEG, -15, 15));
                break;
            case AIM_ANGLE_SCALE:
                turretSubsystem.AIM_ANGLE_SCALE = clamp(value, 0.50, 1.50);
                break;
            case CAMERA_LEFT_OF_SHOOTER_IN:
                turretSubsystem.CAMERA_LEFT_OF_SHOOTER_IN = clamp(value, -24, 24);
                break;
            case CAMERA_HEIGHT_IN:
                turretSubsystem.CAMERA_HEIGHT_IN = clamp(value, 1, 40);
                break;
            case CAMERA_UP_ANGLE_DEG:
                turretSubsystem.CAMERA_UP_ANGLE_DEG = clamp(value, -20, 80);
                break;
            case DEFAULT_TAG_HEIGHT_IN:
                turretSubsystem.DEFAULT_TAG_HEIGHT_IN = clamp(value, 1, 100);
                break;
            case BANK_AIM_CENTER_OFFSET_IN:
                turretSubsystem.BANK_AIM_CENTER_OFFSET_IN = clamp(value, 0, 48);
                break;
            case BANK_AIM_MAX_OFFSET_IN:
                turretSubsystem.BANK_AIM_MAX_OFFSET_IN = clamp(value, 0, 48);
                break;
            case BANK_AIM_FULL_SIDE_IN:
                turretSubsystem.BANK_AIM_FULL_SIDE_IN = clamp(value, 0.25, 48);
                break;
            case BANK_AIM_SWITCH_HYSTERESIS_IN:
                turretSubsystem.BANK_AIM_SWITCH_HYSTERESIS_IN = clamp(value, 0, 24);
                break;
            case BANK_AIM_SWITCH_HOLD_MS:
                turretSubsystem.BANK_AIM_SWITCH_HOLD_MS = clamp(value, 0, 5000);
                break;
            case TURRET_PIVOT_FORWARD_IN:
                OdometryConfig.TURRET_PIVOT_FORWARD_IN = clamp(value, -24, 24);
                break;
            case TURRET_PIVOT_LEFT_IN:
                OdometryConfig.TURRET_PIVOT_LEFT_IN = clamp(value, -24, 24);
                break;
            default:
                throw new IllegalArgumentException("Unknown calibration " + calibration);
        }
        turret.refreshAimCalibration();
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
