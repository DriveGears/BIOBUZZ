package org.firstinspires.ftc.teamcode.fromOldRobot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Deque;
import java.util.List;

@TeleOp(name = "TurretTracking", group = "TurretBot")
public class TurretTeleOp extends LinearOpMode {

    TurretHardware turret = new TurretHardware();

    static final int APRILTAG_PIPELINE_INDEX = 0;
    static final int TARGET_TAG_ID = 34;

    static final long MAX_RESULT_STALENESS_MS = 120;

    static final int PAN_COAST_FRAMES = 6;
    static final double PAN_COAST_DECAY = 0.82;

    // ---------------- Pan gains ----------------
    static final double PAN_MAX_POWER = 0.55;
    static final double PAN_DEADBAND_DEG = 0.30;

    /**
     * Set to 0.0 to aim dead center at the tag.
     * Positive = shift aim left, Negative = shift aim right.
     */
    static final double PAN_AIM_OFFSET_DEG = 0.0;

    // ---------------- Cascade pan control ----------------
    /**
     * Measured from mechanical calibration: 2.04 ticks per degree.
     */
    static final double PAN_TICKS_PER_DEG = 2.04;
    static final double PAN_OUTER_CORRECTION_GAIN = 0.18;

    /** Positive power makes ticks INCREASE. */
    static final double PAN_TICK_SIGN = 1.0;

    // Inner loop
    static final double PAN_INNER_KP = 0.045;
    static final double PAN_INNER_KD = 0.0035;
    static final double PAN_INNER_KV = 0.0015;
    static final double PAN_INNER_KI = 0.015;
    static final double PAN_INNER_MAX_I = 0.10;
    double panInnerIntegral = 0;
    static final double PAN_INNER_STICTION = 0.050;
    static final double PAN_INNER_STICTION_FADE_TICKS = 5.0;
    static final double PAN_INNER_DEADBAND_TICKS = 0.6;

    static final double PAN_DIRECTION_SIGN_DEFAULT = -1.0;
    double panDirectionOverride = PAN_DIRECTION_SIGN_DEFAULT;
    boolean lastXButton = false;

    // ---------------- Latency compensation ----------------
    static final boolean LATENCY_COMP_ENABLED = true;
    static final double PAN_DEGREES_PER_TICK = 0.490;
    static final double PAN_LATENCY_SIGN = -1.0;
    static final double PAN_LATENCY_FUDGE_MS = 10.0;
    static final double PAN_HISTORY_WINDOW_MS = 400.0;

    final Deque<double[]> panTickHistory = new ArrayDeque<>();
    double lastLatencyMs = 0;
    double lastTurretMovedDeg = 0;

    // ---------------- Frame Tracking & Target Velocity ----------------
    double lastLimelightTimestamp = -1.0;
    static final double PAN_TARGET_RATE_SMOOTH = 0.10;
    static final double PAN_RATE_DEADBAND_DPS = 4.0;
    static final double PAN_MAX_TARGET_RATE_DPS = 60.0;
    static final double RATE_MAX_GAP_MS = 180.0;
    double lastBearingDeg = 0;
    double lastBearingTimeMs = -1;
    boolean bearingInitialized = false;
    double targetRateDegPerSec = 0;

    // ---------------- Motion prediction ----------------
    static final double TOF_INTERCEPT_S = 0.12;
    static final double TOF_PER_INCH_S = 0.0055;
    static final double MECH_LATENCY_S = 0.06;
    static final double MAX_LEAD_TIME_S = 1.20;
    static final double MAX_LAUNCH_LEAD_S = 0.40;
    static final double MIN_LEAD_DENOMINATOR = 0.45;

    static final double RANGE_RATE_SMOOTH = 0.25;
    static final double MAX_RANGE_RATE_IN_PER_SEC = 140.0;
    static final double MAX_LEAD_DIST_IN = 36.0;
    static final double MAX_LEAD_DEG = 12.0;

    static final double LEAD_SCALE_STEP = 0.10;
    static final double LEAD_SCALE_MIN = 0.0;
    static final double LEAD_SCALE_MAX = 2.0;
    double leadScale = 1.0;
    boolean lastDpadLeft = false;
    boolean lastDpadRight = false;

    boolean predictionEnabled = false;
    boolean lastAButton = false;

    double rangeRateInPerSec = 0;
    double lastDistanceForRate = 0;
    double lastDistanceTimeMs = -1;
    boolean rangeRateInitialized = false;
    double impactDistanceIn = 0;
    double tofS = 0;
    double timeToImpactS = 0;
    double panLeadDeg = 0;

    double panSetpointTicks = 0;
    double panSetpointVelTicksPerSec = 0;
    boolean panSetpointInitialized = false;
    double lastTickError = 0;

    boolean aimReady = false;
    boolean shotReady = false;
    boolean lastShotReady = false;

    final double[] txHistory = new double[3];
    int txHistoryCount = 0;

    final double[] distanceHistory = new double[5];
    int distanceHistoryCount = 0;

    static final int PAN_REACQUIRE_FRAMES = 1;
    int consecutiveGoodFrames = 0;
    static final int TARGET_LOST_GRACE_FRAMES = 5;
    int consecutiveMissedFrames = 0;

    static final int PAN_MIN_TICKS = -135;
    static final int PAN_MAX_TICKS = 135;

    // Distance geometry
    static final double TURRET_CAMERA_HEIGHT_IN = 48.0;
    static final double TARGET_TAG_HEIGHT_IN = 14.5;
    static final double CAMERA_TILT_DEG = 24.24;
    static final double MIN_VALID_DEPRESSION_ANGLE_DEG = 2.0;
    static final double DISTANCE_EMA_ALPHA = 0.25;
    double filteredDistanceIn = 0;
    boolean distanceFilterInitialized = false;

    // Hood mapping
    static final double HOOD_NEAR_DIST_IN = 24.0;
    static final double HOOD_NEAR_POS = 0.48;
    static final double HOOD_FAR_DIST_IN = 132.0;
    static final double HOOD_FAR_POS = 0.14;

    // ---------------- Flywheel mapping ----------------
    static final double[] FLYWHEEL_TABLE_DIST_IN = {
            38.0, 40.0, 42.0, 58.0, 60.0, 82.0, 90.0, 92.0, 98.0, 120.0, 132.0
    };
    static final double[] FLYWHEEL_TABLE_VEL_TPS = {
            700.0, 700.0, 730.0, 845.0, 865.0, 965.0, 1012.0, 1020.0, 1065.0, 1220.0, 1250.0
    };

    static final double FLYWHEEL_NEAR_DIST_IN = FLYWHEEL_TABLE_DIST_IN[0];
    static final double FLYWHEEL_NEAR_VELOCITY_TPS = FLYWHEEL_TABLE_VEL_TPS[0];
    static final double FLYWHEEL_MAX_DIST_IN =
            FLYWHEEL_TABLE_DIST_IN[FLYWHEEL_TABLE_DIST_IN.length - 1];

    double tunableMaxVelocityTPS = 1250.0;
    static final double FLYWHEEL_MAX_VELOCITY_TPS = 1600.0;
    static final double FLYWHEEL_BASE_VELOCITY_TPS = 700.0;
    static final double RANGE_SENSITIVITY_FACTOR = 2.0;

    // ---------------- Flywheel PIDF ----------------
    static final double FLYWHEEL_MAX_ACHIEVABLE_TPS = 2300.0;
    static final double FLYWHEEL_P = 10.0;
    static final double FLYWHEEL_I = 0.08;
    static final double FLYWHEEL_D = 2.0;
    static final double FLYWHEEL_F = 14.0;

    static final double FLYWHEEL_BRAKE_THRESHOLD_TPS = 46.7;
    static final double FLYWHEEL_BRAKE_FULL_SCALE_TPS = 280.0;
    static final double FLYWHEEL_BRAKE_POWER_MAX = 0.10;
    boolean flywheelBraking = false;

    static final double FLYWHEEL_SETPOINT_DEADBAND_TPS = 5.0;

    double desiredFlywheelTPS = -1;
    double commandedFlywheelTPS = -1;

    static final double FLYWHEEL_SLEW_UP_TPS_PER_SEC = 5000.0;
    static final double FLYWHEEL_SLEW_DOWN_TPS_PER_SEC = 400.0;
    static final double FLYWHEEL_CMD_LEAD_TPS = 250.0;
    static final double FLYWHEEL_RESEND_EPSILON_TPS = 1.0;

    static final double FLYWHEEL_BOOST_ENTER_TPS = 150.0;
    static final double FLYWHEEL_BOOST_EXIT_TPS = 60.0;
    static final double FLYWHEEL_BOOST_MIN_SETPOINT_TPS = 50.0;
    boolean flywheelBoosting = false;
    boolean flywheelOpenLoop = false;

    double flywheelFTrim = 1.0;
    static final double FLYWHEEL_F_TRIM_STEP = 0.05;
    static final double FLYWHEEL_F_TRIM_MIN = 0.50;
    static final double FLYWHEEL_F_TRIM_MAX = 1.50;
    double lastAppliedFTrim = -1;

    double windowMaxOvershootTPS = 0;
    double lastWindowOvershootTPS = 0;

    static final double FLYWHEEL_DIST_QUANTUM_IN = 2.0;
    static final double FLYWHEEL_DIST_HYSTERESIS_IN = 1.5;
    double quantizedDistanceIn = -1;

    boolean flywheelSetpointLocked = false;
    boolean lastLeftBumper = false;

    // ---------------- Flywheel readiness ----------------
    static final double FLYWHEEL_READY_TOLERANCE_TPS = 15.0;
    static final int FLYWHEEL_READY_STABLE_LOOPS = 8;
    int flywheelStableLoops = 0;
    boolean flywheelReady = false;
    boolean lastFlywheelReady = false;

    static final double VELOCITY_WINDOW_MS = 1000.0;
    double windowMinVelocity = Double.MAX_VALUE;
    double windowMaxVelocity = -Double.MAX_VALUE;
    double lastWindowSpreadTPS = 0;
    final ElapsedTime velocityWindowTimer = new ElapsedTime();

    static final double FLYWHEEL_MANUAL_STEP_TPS = 25.0;
    double flywheelManualOverrideTPS = FLYWHEEL_BASE_VELOCITY_TPS;
    boolean lastDpadUp = false;
    boolean lastDpadDown = false;
    boolean flywheelManualMode = false;
    // Toggle manual flywheel with START + Y (Y+dpad stays tuning).
    boolean lastYButton = false;
    double previousPanPower = 0;
    int aimStableFrames = 0;
    static final int AIM_STABLE_NEW_FRAMES = 3;
    static final long MAX_HOLD_WITHOUT_TARGET_MS = 350;
    double lastTargetSeenMs = -1;
    double filteredTx = 0;
    boolean filteredTxInitialized = false;

    double flywheelMaxObservedTPS = 0;

    // Rolling screen buffer
    static final int SCREEN_CSV_LINES = 4;
    final Deque<String> liveCsvLines = new ArrayDeque<>();
    static final double TELEMETRY_INTERVAL_MS = 100.0;
    final ElapsedTime telemetryTimer = new ElapsedTime();

    // Disk logging
    static final String LOG_DIR = "/sdcard/FIRST/";
    static final String LOG_PREFIX = "turret_log_";
    static final int LOG_FLUSH_EVERY_N_LOOPS = 10;
    String logFileName = "(none)";
    String logPath = "(not opened)";
    String logError = "";

    BufferedWriter logWriter;
    int logLoopCounter = 0;
    final ElapsedTime logTimer = new ElapsedTime();
    final ElapsedTime loopTimer = new ElapsedTime();

    List<LynxModule> allHubs;

    @Override
    public void runOpMode() {
        telemetry.setDisplayFormat(org.firstinspires.ftc.robotcore.external.Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);

        turret.init(hardwareMap);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        turret.flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turret.limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
        turret.limelight.setPollRateHz(100);
        turret.limelight.start();
        turret.flywheelMotor.setVelocityPIDFCoefficients(
                FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);

        openLogFile();

        telemetry.addLine("=== READY ===");
        telemetry.addData("LOG", logWriter != null ? logPath : ("FAILED - " + logError));
        telemetry.addLine("LB: lock setpoint | RB: measure flywheel max");
        telemetry.addLine("Y+dpad: tune far velocity | dpad up/dn: manual velocity");
        telemetry.addLine("B+dpad: F trim | X: flip pan direction");
        telemetry.addLine("A: toggle lead prediction | dpad L/R: lead scale");
        telemetry.update();
        waitForStart();

        logTimer.reset();
        loopTimer.reset();
        velocityWindowTimer.reset();
        telemetryTimer.reset();

        try {
            while (opModeIsActive()) {
                for (LynxModule hub : allHubs) {
                    hub.clearBulkCache();
                }

                double dtS = loopTimer.seconds();
                loopTimer.reset();
                double dtMs = dtS * 1000.0;
                double nowMs = logTimer.milliseconds();

                // ---------------- FLYWHEEL MAX TEST (right bumper) ----------------
                if (gamepad1.right_bumper) {
                    turret.panMotor.setPower(0);
                    ensureFlywheelOpenLoop(true);
                    turret.flywheelMotor.setPower(1.0);
                    double testVel = turret.flywheelMotor.getVelocity();
                    flywheelMaxObservedTPS = Math.max(flywheelMaxObservedTPS, testVel);
                    desiredFlywheelTPS = -1;
                    commandedFlywheelTPS = -1;
                    flywheelBoosting = false;
                    lastLeftBumper = gamepad1.left_bumper;

                    telemetry.addLine(">>> FLYWHEEL MAX TEST (right bumper)");
                    telemetry.addData("Now", "%.0f tps", testVel);
                    telemetry.addData("PEAK -- report this", "%.0f tps", flywheelMaxObservedTPS);
                    telemetry.addLine("Hold until PEAK stops climbing.");
                    telemetry.update();
                    continue;
                }

                // ---------------- SETPOINT LOCK (left bumper) ----------------
                if (gamepad1.left_bumper && !lastLeftBumper) {
                    flywheelSetpointLocked = !flywheelSetpointLocked;
                }
                lastLeftBumper = gamepad1.left_bumper;

                boolean yHeld = gamepad1.y;
                boolean bHeld = gamepad1.b;

                if (bHeld) {
                    if (gamepad1.dpad_up && !lastDpadUp) {
                        flywheelFTrim = Math.min(FLYWHEEL_F_TRIM_MAX,
                                flywheelFTrim + FLYWHEEL_F_TRIM_STEP);
                    }
                    if (gamepad1.dpad_down && !lastDpadDown) {
                        flywheelFTrim = Math.max(FLYWHEEL_F_TRIM_MIN,
                                flywheelFTrim - FLYWHEEL_F_TRIM_STEP);
                    }
                } else if (yHeld) {
                    if (gamepad1.dpad_up && !lastDpadUp) {
                        tunableMaxVelocityTPS = Math.min(FLYWHEEL_MAX_VELOCITY_TPS,
                                tunableMaxVelocityTPS + FLYWHEEL_MANUAL_STEP_TPS);
                    }
                    if (gamepad1.dpad_down && !lastDpadDown) {
                        tunableMaxVelocityTPS = Math.max(FLYWHEEL_NEAR_VELOCITY_TPS + 50.0,
                                tunableMaxVelocityTPS - FLYWHEEL_MANUAL_STEP_TPS);
                    }
                    commandFlywheel(tunableMaxVelocityTPS, true);
                } else {
                    if (gamepad1.dpad_up && !lastDpadUp) {
                        flywheelManualMode = true;
                        flywheelManualOverrideTPS = Math.min(FLYWHEEL_MAX_VELOCITY_TPS,
                                flywheelManualOverrideTPS + FLYWHEEL_MANUAL_STEP_TPS);
                    }
                    if (gamepad1.dpad_down && !lastDpadDown) {
                        flywheelManualMode = true;
                        flywheelManualOverrideTPS = Math.max(0,
                                flywheelManualOverrideTPS - FLYWHEEL_MANUAL_STEP_TPS);
                    }
                }

                lastDpadUp = gamepad1.dpad_up;
                lastDpadDown = gamepad1.dpad_down;

                if (gamepad1.y && !lastYButton && gamepad1.start) {
                    flywheelManualMode = !flywheelManualMode;
                }
                lastYButton = gamepad1.y;

                if (gamepad1.x && !lastXButton) {
                    panDirectionOverride = -panDirectionOverride;
                    bearingInitialized = false;
                    lastBearingTimeMs = -1;
                    targetRateDegPerSec = 0;
                    panSetpointInitialized = false;
                }
                lastXButton = gamepad1.x;

                if (gamepad1.a && !lastAButton) {
                    predictionEnabled = !predictionEnabled;
                    if (!predictionEnabled) {
                        panLeadDeg = 0;
                        targetRateDegPerSec = 0;
                    }
                }
                lastAButton = gamepad1.a;

                if (gamepad1.dpad_right && !lastDpadRight) {
                    leadScale = Math.min(LEAD_SCALE_MAX, leadScale + LEAD_SCALE_STEP);
                }
                if (gamepad1.dpad_left && !lastDpadLeft) {
                    leadScale = Math.max(LEAD_SCALE_MIN, leadScale - LEAD_SCALE_STEP);
                }
                lastDpadRight = gamepad1.dpad_right;
                lastDpadLeft = gamepad1.dpad_left;

                LLResult result = turret.limelight.getLatestResult();
                boolean fresh = result != null && result.isValid()
                        && result.getStaleness() >= 0
                        && result.getStaleness() < MAX_RESULT_STALENESS_MS;

                double frameTimestamp = (result != null) ? result.getTimestamp() : -1.0;
                boolean isNewFrame = fresh && (frameTimestamp != lastLimelightTimestamp);
                if (isNewFrame) {
                    lastLimelightTimestamp = frameTimestamp;
                }

                LLResultTypes.FiducialResult target = null;
                if (fresh) {
                    List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                    if (tags != null) {
                        for (LLResultTypes.FiducialResult tag : tags) {
                            if (TARGET_TAG_ID < 0 || tag.getFiducialId() == TARGET_TAG_ID) {
                                target = tag;
                                break;
                            }
                        }
                    }
                }

                double rawTx = 0;
                double tx = 0;
                double distanceIn = FLYWHEEL_NEAR_DIST_IN;
                double appliedPanPower = 0;

                int currentPanTicks = turret.panMotor.getCurrentPosition();
                double panMotorVelocity = turret.panMotor.getVelocity();

                panTickHistory.addLast(new double[]{nowMs, currentPanTicks});
                while (!panTickHistory.isEmpty()
                        && nowMs - panTickHistory.peekFirst()[0] > PAN_HISTORY_WINDOW_MS) {
                    panTickHistory.removeFirst();
                }

                if (target != null) {
                    consecutiveMissedFrames = 0;
                    if (isNewFrame) {
                        consecutiveGoodFrames++;
                        lastTargetSeenMs = nowMs;
                    }

                    rawTx = target.getTargetXDegrees();
                    if (isNewFrame || !filteredTxInitialized) {
                        filteredTx = medianFilterTx(rawTx);
                        filteredTxInitialized = true;
                    }
                    tx = filteredTx;

                    double ty = target.getTargetYDegrees();
                    double totalAngleDeg = CAMERA_TILT_DEG - ty;
                    double heightDiff = TURRET_CAMERA_HEIGHT_IN - TARGET_TAG_HEIGHT_IN;

                    boolean distanceValid = totalAngleDeg > MIN_VALID_DEPRESSION_ANGLE_DEG;
                    if (distanceValid && isNewFrame) {
                        double rawDistanceIn = heightDiff / Math.tan(Math.toRadians(totalAngleDeg));
                        if (rawDistanceIn > 0 && rawDistanceIn < 200.0) {
                            double filteredRawDist = medianFilterDistance(rawDistanceIn);
                            if (!distanceFilterInitialized) {
                                filteredDistanceIn = filteredRawDist;
                                distanceFilterInitialized = true;
                            } else {
                                filteredDistanceIn +=
                                        DISTANCE_EMA_ALPHA * (filteredRawDist - filteredDistanceIn);
                            }
                        }
                    }
                    distanceIn = distanceFilterInitialized ? filteredDistanceIn : FLYWHEEL_NEAR_DIST_IN;

                    // ---- LATENCY COMPENSATION ----
                    double totalLatencyMs = result.getStaleness()
                            + result.getCaptureLatency()
                            + result.getTargetingLatency()
                            + PAN_LATENCY_FUDGE_MS;
                    lastLatencyMs = totalLatencyMs;
                    double turretMovedDeg = 0;
                    double ticksAtCapture = currentPanTicks;
                    if (LATENCY_COMP_ENABLED) {
                        ticksAtCapture = panTicksAt(nowMs - totalLatencyMs);
                        turretMovedDeg = (currentPanTicks - ticksAtCapture) * PAN_DEGREES_PER_TICK;
                    }
                    lastTurretMovedDeg = turretMovedDeg;

                    // ---- TARGET ANGULAR RATE (Field-Frame Bearing) ----
                    double bearingAtCaptureDeg = LATENCY_COMP_ENABLED
                            ? tx + ticksAtCapture * PAN_DEGREES_PER_TICK * PAN_LATENCY_SIGN
                            : tx;

                    if (isNewFrame) {
                        double bearingGapMs = (lastBearingTimeMs < 0) ? -1 : (nowMs - lastBearingTimeMs);
                        if (bearingInitialized && bearingGapMs > 1.0 && bearingGapMs <= RATE_MAX_GAP_MS) {
                            double rawRate = clamp(
                                    (bearingAtCaptureDeg - lastBearingDeg) / (bearingGapMs / 1000.0),
                                    -PAN_MAX_TARGET_RATE_DPS, PAN_MAX_TARGET_RATE_DPS);
                            if (Math.abs(rawRate) < PAN_RATE_DEADBAND_DPS) {
                                rawRate = 0;
                            }
                            targetRateDegPerSec +=
                                    PAN_TARGET_RATE_SMOOTH * (rawRate - targetRateDegPerSec);
                        } else if (bearingGapMs > RATE_MAX_GAP_MS) {
                            targetRateDegPerSec = 0;
                            bearingInitialized = true;
                        } else {
                            bearingInitialized = true;
                        }
                        lastBearingDeg = bearingAtCaptureDeg;
                        lastBearingTimeMs = nowMs;

                        // ---- RANGE RATE ----
                        double distGapMs = (lastDistanceTimeMs < 0) ? -1 : (nowMs - lastDistanceTimeMs);
                        if (rangeRateInitialized && distGapMs > 1.0 && distGapMs <= RATE_MAX_GAP_MS) {
                            double rawRangeRate = clamp(
                                    (distanceIn - lastDistanceForRate) / (distGapMs / 1000.0),
                                    -MAX_RANGE_RATE_IN_PER_SEC, MAX_RANGE_RATE_IN_PER_SEC);
                            rangeRateInPerSec +=
                                    RANGE_RATE_SMOOTH * (rawRangeRate - rangeRateInPerSec);
                        } else if (distGapMs > RATE_MAX_GAP_MS) {
                            rangeRateInPerSec = 0;
                            rangeRateInitialized = true;
                        } else {
                            rangeRateInitialized = true;
                        }
                        lastDistanceForRate = distanceIn;
                        lastDistanceTimeMs = nowMs;
                    }

                    // ---- INTERCEPT SOLVE ----
                    double launchLeadS = clamp(MECH_LATENCY_S + totalLatencyMs / 1000.0,
                            0.0, MAX_LAUNCH_LEAD_S);
                    double leadGain = predictionEnabled ? leadScale : 0.0;
                    double effRangeRate = rangeRateInPerSec * leadGain;

                    double denom = Math.max(1.0 - TOF_PER_INCH_S * effRangeRate,
                            MIN_LEAD_DENOMINATOR);
                    timeToImpactS = clamp(
                            (launchLeadS + TOF_INTERCEPT_S + TOF_PER_INCH_S * distanceIn) / denom,
                            0.0, MAX_LEAD_TIME_S);
                    tofS = Math.max(timeToImpactS - launchLeadS, 0.05);

                    double distLeadIn = clamp(effRangeRate * timeToImpactS,
                            -MAX_LEAD_DIST_IN, MAX_LEAD_DIST_IN);
                    impactDistanceIn = clamp(distanceIn + distLeadIn, 12.0, 200.0);

                    // Continuous rate thresholding to prevent step discontinuities
                    double effectiveRate = 0.0;
                    if (predictionEnabled && Math.abs(targetRateDegPerSec) > PAN_RATE_DEADBAND_DPS) {
                        effectiveRate = targetRateDegPerSec
                                - Math.copySign(PAN_RATE_DEADBAND_DPS, targetRateDegPerSec);
                    }

                    panLeadDeg = clamp(effectiveRate * timeToImpactS * leadGain,
                            -MAX_LEAD_DEG, MAX_LEAD_DEG);

                    quantizeDistance(impactDistanceIn);

                    double panErrorDeg = (tx - PAN_AIM_OFFSET_DEG)
                            + turretMovedDeg
                            + panLeadDeg;

                    if (isNewFrame) {
                        aimStableFrames = Math.abs(panErrorDeg) <= PAN_DEADBAND_DEG
                                ? aimStableFrames + 1 : 0;
                    }
                    aimReady = aimStableFrames >= AIM_STABLE_NEW_FRAMES;

                    // ---- OUTER CASCADE LOOP ----
                    if (consecutiveGoodFrames >= PAN_REACQUIRE_FRAMES) {
                        double impliedTargetTicks = currentPanTicks
                                + panDirectionOverride * panErrorDeg * PAN_TICKS_PER_DEG;

                        if (!panSetpointInitialized) {
                            panSetpointTicks = impliedTargetTicks;
                            panSetpointInitialized = true;
                        } else {
                            panSetpointTicks += PAN_OUTER_CORRECTION_GAIN
                                    * (impliedTargetTicks - panSetpointTicks);
                        }
                    }

                    setHoodForDistance(impactDistanceIn);

                    if (!yHeld) {
                        if (flywheelManualMode) {
                            commandFlywheel(flywheelManualOverrideTPS, false);
                        } else {
                            setFlywheelForDistance(impactDistanceIn);
                        }
                    }

                } else {
                    consecutiveMissedFrames++;
                    consecutiveGoodFrames = 0;

                    if (!yHeld) {
                        double fallbackTPS = (desiredFlywheelTPS > 0)
                                ? desiredFlywheelTPS : flywheelManualOverrideTPS;
                        commandFlywheel(fallbackTPS, false);
                    }

                    if (consecutiveMissedFrames > PAN_COAST_FRAMES) {
                        targetRateDegPerSec *= PAN_COAST_DECAY;
                    }
                    aimReady = false;
                    aimStableFrames = 0;
                    panLeadDeg = 0;
                    impactDistanceIn = distanceIn;

                    if (consecutiveMissedFrames >= TARGET_LOST_GRACE_FRAMES) {
                        txHistoryCount = 0;
                        distanceHistoryCount = 0;
                        bearingInitialized = false;
                        lastBearingTimeMs = -1;
                        targetRateDegPerSec = 0;
                        panInnerIntegral = 0;
                        rangeRateInitialized = false;
                        lastDistanceTimeMs = -1;
                        rangeRateInPerSec = 0;
                        filteredTxInitialized = false;
                    }
                }

                // ---------------- PAN INNER LOOP ----------------
                double targetRateFeedforward = (predictionEnabled && Math.abs(targetRateDegPerSec) > PAN_RATE_DEADBAND_DPS)
                        ? (targetRateDegPerSec - Math.copySign(PAN_RATE_DEADBAND_DPS, targetRateDegPerSec)) : 0.0;
                panSetpointVelTicksPerSec = panDirectionOverride
                        * targetRateFeedforward * PAN_TICKS_PER_DEG;

                if (!panSetpointInitialized) {
                    panSetpointTicks = currentPanTicks;
                }
                // Hold the current position when the vision target has been missing too long.
                if (lastTargetSeenMs < 0 || nowMs - lastTargetSeenMs > MAX_HOLD_WITHOUT_TARGET_MS) {
                    panSetpointTicks = currentPanTicks;
                    panSetpointVelTicksPerSec = 0;
                    panInnerIntegral = 0;
                }
                panSetpointTicks = clamp(panSetpointTicks, PAN_MIN_TICKS, PAN_MAX_TICKS);

                double tickError = panSetpointTicks - currentPanTicks;
                lastTickError = tickError;

                double innerPower;
                if (Math.abs(tickError) < PAN_INNER_DEADBAND_TICKS
                        && Math.abs(panSetpointVelTicksPerSec) < 2.0) {
                    innerPower = 0;
                    panInnerIntegral *= 0.90;
                } else {
                    double stiction = Math.copySign(
                            PAN_INNER_STICTION * clamp(
                                    Math.abs(tickError) / PAN_INNER_STICTION_FADE_TICKS,
                                    0.0, 1.0),
                            tickError);

                    if (Math.abs(previousPanPower) < PAN_MAX_POWER * 0.95
                            || Math.signum(tickError) != Math.signum(previousPanPower)) {
                        panInnerIntegral += PAN_INNER_KI * tickError * dtS;
                        panInnerIntegral = clamp(panInnerIntegral,
                                -PAN_INNER_MAX_I, PAN_INNER_MAX_I);
                    }

                    double velError = panSetpointVelTicksPerSec - panMotorVelocity;

                    innerPower = PAN_INNER_KP * tickError
                            + PAN_INNER_KV * panSetpointVelTicksPerSec
                            + PAN_INNER_KD * velError
                            + panInnerIntegral
                            + stiction;

                    innerPower *= PAN_TICK_SIGN;
                }

                appliedPanPower = clamp(innerPower, -PAN_MAX_POWER, PAN_MAX_POWER);

                double tickDirection = PAN_TICK_SIGN * appliedPanPower;
                if (currentPanTicks >= PAN_MAX_TICKS && tickDirection > 0) appliedPanPower = 0;
                if (currentPanTicks <= PAN_MIN_TICKS && tickDirection < 0) appliedPanPower = 0;
                turret.panMotor.setPower(appliedPanPower);
                previousPanPower = appliedPanPower;

                // ---------------- FLYWHEEL ----------------
                applyFlywheelPIDF();
                updateFlywheelCommand(dtS);

                double flywheelVelocity = turret.flywheelMotor.getVelocity();

                if (desiredFlywheelTPS > 0) {
                    windowMaxOvershootTPS = Math.max(windowMaxOvershootTPS,
                            flywheelVelocity - desiredFlywheelTPS);
                }

                if (desiredFlywheelTPS > 0
                        && Math.abs(flywheelVelocity - desiredFlywheelTPS) < FLYWHEEL_READY_TOLERANCE_TPS) {
                    flywheelStableLoops++;
                } else {
                    flywheelStableLoops = 0;
                }
                flywheelReady = flywheelStableLoops >= FLYWHEEL_READY_STABLE_LOOPS;
                shotReady = flywheelReady && aimReady && target != null && fresh;

                if (shotReady && !lastShotReady) {
                    gamepad1.rumbleBlips(1);
                }
                lastShotReady = shotReady;
                lastFlywheelReady = flywheelReady;

                windowMinVelocity = Math.min(windowMinVelocity, flywheelVelocity);
                windowMaxVelocity = Math.max(windowMaxVelocity, flywheelVelocity);
                if (velocityWindowTimer.milliseconds() >= VELOCITY_WINDOW_MS) {
                    lastWindowSpreadTPS = windowMaxVelocity - windowMinVelocity;
                    lastWindowOvershootTPS = windowMaxOvershootTPS;
                    windowMinVelocity = Double.MAX_VALUE;
                    windowMaxVelocity = -Double.MAX_VALUE;
                    windowMaxOvershootTPS = 0;
                    velocityWindowTimer.reset();
                }

                double predictedRangeErrorIn = 0;
                if (commandedFlywheelTPS > 0) {
                    predictedRangeErrorIn = RANGE_SENSITIVITY_FACTOR * distanceIn
                            * (lastWindowSpreadTPS / commandedFlywheelTPS);
                }

                String csvLine = String.format(java.util.Locale.US,
                        "%.0f,%.1f,%d,%+.2f,%+.2f,%+.2f,%+.0f,%+.1f,%.0f,%+.2f,%d,%.1f,%+.2f,%.1f,%+.1f,%.1f,%+.2f,%.3f,%.0f,%.0f,%d,%d",
                        nowMs, dtMs, (target != null ? 1 : 0), rawTx, tx, appliedPanPower,
                        panMotorVelocity, targetRateDegPerSec, lastLatencyMs, lastTurretMovedDeg,
                        currentPanTicks, panSetpointTicks, lastTickError,
                        distanceIn, rangeRateInPerSec, impactDistanceIn,
                        panLeadDeg, timeToImpactS, commandedFlywheelTPS, flywheelVelocity,
                        shotReady ? 1 : 0, flywheelBoosting ? 1 : 0);

                logDisk(csvLine);

                if (telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS) {
                    telemetryTimer.reset();

                    liveCsvLines.addLast(csvLine);
                    while (liveCsvLines.size() > SCREEN_CSV_LINES) {
                        liveCsvLines.removeFirst();
                    }

                    telemetry.addLine(shotReady ? ">>> READY TO SHOOT <<<"
                            : (flywheelReady ? "... aiming ..." : "... spinning up ..."));
                    telemetry.addData("FW", "goal %.0f / cmd %.0f / actual %.0f%s%s%s",
                            desiredFlywheelTPS, commandedFlywheelTPS, flywheelVelocity,
                            flywheelSetpointLocked ? " [LOCKED]" : "",
                            flywheelBoosting ? " [BOOST]" : "",
                            flywheelBraking ? " [BRAKING]" : "");
                    telemetry.addData("Cascade", "setpt %.0f / actual %d / err %+.1f tk",
                            panSetpointTicks, currentPanTicks, lastTickError);
                    telemetry.addData("Vision", "%s  miss=%d  lat=%.0f ms",
                            target != null ? "TRACKING" : "NO TARGET",
                            consecutiveMissedFrames, lastLatencyMs);
                    telemetry.addData("Pan", "%d ticks (%.1f deg)%s",
                            currentPanTicks, currentPanTicks * PAN_DEGREES_PER_TICK,
                            (currentPanTicks >= PAN_MAX_TICKS || currentPanTicks <= PAN_MIN_TICKS)
                                    ? "  *** AT SOFT LIMIT ***" : "");
                    telemetry.addData("Features", "latency=%s lead=%s x%.1f",
                            LATENCY_COMP_ENABLED ? "ON" : "off",
                            predictionEnabled ? "ON" : "off", leadScale);
                    telemetry.addData("LEAD (A toggles)", "%s x%.1f  T=%.2fs (tof %.2f)",
                            predictionEnabled ? "ON" : "OFF", leadScale, timeToImpactS, tofS);
                    telemetry.addData("Dist", "now %.1f -> impact %.1f in (bucket %.0f)",
                            distanceIn, impactDistanceIn, quantizedDistanceIn);
                    telemetry.addData("Target closing", "%+.1f in/s", rangeRateInPerSec);
                    telemetry.addData("Sweep", "%+.1f deg/s -> aim ahead %+.2f deg",
                            targetRateDegPerSec, panLeadDeg);
                    telemetry.addData("Overshoot (1s)", "%+.0f tps", lastWindowOvershootTPS);
                    telemetry.addData("F trim (hold B + dpad)", "%.2f -> F=%.2f",
                            flywheelFTrim, FLYWHEEL_F * flywheelFTrim);
                    telemetry.addData("FW toggle", "START+Y; Y+dpad tunes max");
                    telemetry.addData("FW mode", flywheelManualMode
                            ? "MANUAL %.0f -- press Y for auto (table IGNORED)"
                            : "AUTO from distance table", flywheelManualOverrideTPS);
                    telemetry.addData("Spread (1s)", "%.0f tps -> ~%.1f in of range",
                            lastWindowSpreadTPS, predictedRangeErrorIn);
                    telemetry.addData("Latency", "%.0f ms -> %+.2f deg lead",
                            lastLatencyMs, lastTurretMovedDeg);
                    telemetry.addData("Loop", "%.1f ms", dtMs);
                    telemetry.addData("Max Velocity", "%.0f TPS", tunableMaxVelocityTPS);
                    telemetry.addData("Log", logWriter != null ? logPath : ("FAILED " + logError));
                    telemetry.addLine("t,dt,ok,tx,flt,pwr,pvel,trate,lat,mvd,tick,setpt,terr,dist,rrate,idist,plead,T,cmd,fw,rdy,bst");
                    for (String line : liveCsvLines) {
                        telemetry.addLine(line);
                    }
                    telemetry.update();
                }
            }
        } finally {
            turret.panMotor.setPower(0);
            turret.flywheelMotor.setPower(0);
            turret.limelight.stop();
            closeLogFile();
        }
    }

    private double panTicksAt(double timeMs) {
        double[] prev = null;
        for (double[] sample : panTickHistory) {
            if (sample[0] >= timeMs) {
                if (prev == null) return sample[1];
                double span = Math.max(sample[0] - prev[0], 1e-6);
                double t = (timeMs - prev[0]) / span;
                return prev[1] + t * (sample[1] - prev[1]);
            }
            prev = sample;
        }
        return prev != null ? prev[1] : 0;
    }

    private void openLogFile() {
        String stamp = new java.text.SimpleDateFormat("MMdd_HHmmss", java.util.Locale.US)
                .format(new java.util.Date());
        logFileName = LOG_PREFIX + stamp + ".csv";

        File[] candidates = new File[] {
                new File(LOG_DIR),
                hardwareMap.appContext.getExternalFilesDir(null),
                hardwareMap.appContext.getFilesDir()
        };

        for (File dir : candidates) {
            if (dir == null) continue;
            try {
                if (!dir.exists() && !dir.mkdirs()) {
                    logError = "mkdirs failed: " + dir.getPath();
                    continue;
                }
                File file = new File(dir, logFileName);
                logWriter = new BufferedWriter(new FileWriter(file, false));
                logWriter.write("time_ms,dt_ms,fresh,raw_tx,filtered_tx,applied_power,"
                        + "pan_velocity,target_rate_dps,latency_ms,moved_deg,pan_ticks,"
                        + "pan_setpoint_ticks,tick_error,distance_in,"
                        + "range_rate_ips,impact_dist_in,pan_lead_deg,time_to_impact_s,"
                        + "flywheel_cmd_tps,flywheel_actual_tps,shot_ready,flywheel_boost");
                logWriter.newLine();
                logWriter.flush();
                logPath = file.getAbsolutePath();
                logError = "";
                return;
            } catch (Exception e) {
                logWriter = null;
                logError = dir.getPath() + " -> " + e.getClass().getSimpleName()
                        + ": " + e.getMessage();
            }
        }
        logPath = "FAILED";
    }

    private void logDisk(String line) {
        if (logWriter == null) return;
        try {
            logWriter.write(line);
            logWriter.newLine();
            logLoopCounter++;
            if (logLoopCounter >= LOG_FLUSH_EVERY_N_LOOPS) {
                logLoopCounter = 0;
                logWriter.flush();
            }
        } catch (IOException ignored) {}
    }

    private void closeLogFile() {
        if (logWriter == null) return;
        try {
            logWriter.flush();
            logWriter.close();
        } catch (IOException ignored) {}
    }

    private double medianFilterTx(double newTx) {
        for (int i = txHistory.length - 1; i > 0; i--) {
            txHistory[i] = txHistory[i - 1];
        }
        txHistory[0] = newTx;

        if (txHistoryCount < txHistory.length) {
            txHistoryCount++;
        }

        double[] sorted = new double[txHistoryCount];
        System.arraycopy(txHistory, 0, sorted, 0, txHistoryCount);
        Arrays.sort(sorted);
        return sorted[sorted.length / 2];
    }

    private double medianFilterDistance(double newDist) {
        for (int i = distanceHistory.length - 1; i > 0; i--) {
            distanceHistory[i] = distanceHistory[i - 1];
        }
        distanceHistory[0] = newDist;

        if (distanceHistoryCount < distanceHistory.length) {
            distanceHistoryCount++;
        }

        double[] sorted = new double[distanceHistoryCount];
        System.arraycopy(distanceHistory, 0, sorted, 0, distanceHistoryCount);
        Arrays.sort(sorted);
        return sorted[sorted.length / 2];
    }

    private void setHoodForDistance(double distanceIn) {
        double ratio = (distanceIn - HOOD_NEAR_DIST_IN) / (HOOD_FAR_DIST_IN - HOOD_NEAR_DIST_IN);
        ratio = clamp(ratio, 0.0, 1.0);
        double hoodPos = HOOD_NEAR_POS + ratio * (HOOD_FAR_POS - HOOD_NEAR_POS);
        turret.hoodServo.setPosition(clamp(hoodPos, 0.0, 1.0));
    }

    private double quantizeDistance(double distanceIn) {
        if (quantizedDistanceIn < 0
                || Math.abs(distanceIn - quantizedDistanceIn) > FLYWHEEL_DIST_HYSTERESIS_IN) {
            quantizedDistanceIn = Math.round(distanceIn / FLYWHEEL_DIST_QUANTUM_IN)
                    * FLYWHEEL_DIST_QUANTUM_IN;
        }
        return quantizedDistanceIn;
    }

    private double flywheelVelocityForDistance(double distanceIn) {
        int last = FLYWHEEL_TABLE_DIST_IN.length - 1;

        if (distanceIn <= FLYWHEEL_TABLE_DIST_IN[0]) {
            return FLYWHEEL_TABLE_VEL_TPS[0];
        }
        if (distanceIn >= FLYWHEEL_TABLE_DIST_IN[last]) {
            return tunableMaxVelocityTPS;
        }

        for (int i = 0; i < last; i++) {
            double d0 = FLYWHEEL_TABLE_DIST_IN[i];
            double d1 = FLYWHEEL_TABLE_DIST_IN[i + 1];
            if (distanceIn >= d0 && distanceIn <= d1) {
                double v0 = FLYWHEEL_TABLE_VEL_TPS[i];
                double v1 = (i + 1 == last) ? tunableMaxVelocityTPS
                        : FLYWHEEL_TABLE_VEL_TPS[i + 1];
                double t = (distanceIn - d0) / (d1 - d0);
                return v0 + t * (v1 - v0);
            }
        }

        return tunableMaxVelocityTPS;
    }

    private void setFlywheelForDistance(double distanceIn) {
        double stableDistance = quantizedDistanceIn >= 0 ? quantizedDistanceIn : distanceIn;
        commandFlywheel(flywheelVelocityForDistance(stableDistance), false);
    }

    private void commandFlywheel(double desiredTPS, boolean overrideLock) {
        if (flywheelSetpointLocked && !overrideLock && desiredFlywheelTPS >= 0) {
            return;
        }

        double target = clamp(desiredTPS, 0.0, FLYWHEEL_MAX_VELOCITY_TPS);
        if (desiredFlywheelTPS < 0
                || Math.abs(target - desiredFlywheelTPS) > FLYWHEEL_SETPOINT_DEADBAND_TPS) {
            desiredFlywheelTPS = target;
        }
    }

    private void ensureFlywheelOpenLoop(boolean open) {
        if (open == flywheelOpenLoop) return;
        flywheelOpenLoop = open;
        if (open) {
            turret.flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            turret.flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lastAppliedFTrim = -1;
            applyFlywheelPIDF();
        }
    }

    private void updateFlywheelCommand(double dtS) {
        if (desiredFlywheelTPS < 0) return;

        double actual = turret.flywheelMotor.getVelocity();

        if (commandedFlywheelTPS < 0) {
            commandedFlywheelTPS = Math.max(0, actual);
        }

        double slewRate = (desiredFlywheelTPS > commandedFlywheelTPS)
                ? FLYWHEEL_SLEW_UP_TPS_PER_SEC
                : FLYWHEEL_SLEW_DOWN_TPS_PER_SEC;
        double maxStep = slewRate * Math.max(dtS, 0.001);
        double error = desiredFlywheelTPS - commandedFlywheelTPS;
        commandedFlywheelTPS += clamp(error, -maxStep, maxStep);

        double leadCap = Math.max(0, actual) + FLYWHEEL_CMD_LEAD_TPS;
        commandedFlywheelTPS = Math.min(commandedFlywheelTPS, leadCap);
        commandedFlywheelTPS = clamp(commandedFlywheelTPS, 0.0, desiredFlywheelTPS);

        double overshoot = actual - desiredFlywheelTPS;

        if (overshoot > FLYWHEEL_BRAKE_THRESHOLD_TPS) {
            flywheelBoosting = false;
            flywheelBraking = true;
            // Coast during overshoot instead of applying reverse torque.
            ensureFlywheelOpenLoop(true);
            turret.flywheelMotor.setPower(0.0);
            return;
        }

        flywheelBraking = false;

        double gap = desiredFlywheelTPS - actual;
        if (!flywheelBoosting && gap > FLYWHEEL_BOOST_ENTER_TPS) {
            flywheelBoosting = true;
        } else if (flywheelBoosting && gap < FLYWHEEL_BOOST_EXIT_TPS) {
            flywheelBoosting = false;
        }

        if (flywheelBoosting && desiredFlywheelTPS > FLYWHEEL_BOOST_MIN_SETPOINT_TPS) {
            ensureFlywheelOpenLoop(true);
            turret.flywheelMotor.setPower(1.0);
        } else {
            ensureFlywheelOpenLoop(false);
            turret.flywheelMotor.setVelocity(commandedFlywheelTPS);
        }
    }

    private void applyFlywheelPIDF() {
        if (flywheelOpenLoop) return;
        if (flywheelFTrim == lastAppliedFTrim) return;
        lastAppliedFTrim = flywheelFTrim;

        double f = FLYWHEEL_F * flywheelFTrim;
        turret.flywheelMotor.setVelocityPIDFCoefficients(
                FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, f);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
