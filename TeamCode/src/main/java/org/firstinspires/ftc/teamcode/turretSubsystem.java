package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Date;
import java.util.Deque;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "turret_test")
public class turretSubsystem extends LinearOpMode {
    private final ElapsedTime pidTimer = new ElapsedTime();

    Hardware robot;
    DcMotorEx turret;
    Limelight3A limelight;

    static final int APRILTAG_PIPELINE_INDEX = 1;
    static final int[] TARGET_TAG_IDS = {42,45};

    static final long MAX_RESULT_STALENESS_MS = 120;

    static final int PAN_COAST_FRAMES = 6;
    static final double PAN_COAST_DECAY = 0.82;

    // ---------------- Pan gains ----------------
    static final double PAN_MAX_POWER = 0.40;
    static final double PAN_DEADBAND_DEG = 0.50;

    /**
     * Set to 0.0 to aim dead center at the tag.
     * Positive = shift aim left, Negative = shift aim right.
     */
    static final double PAN_AIM_OFFSET_DEG = 0.0;

    // ---------------- Cascade pan control ----------------
    static final double PAN_TICKS_PER_DEG = 2.04;
    static final double PAN_OUTER_CORRECTION_GAIN = 0.05;
    static final double PAN_TICK_SIGN = 1.0;

    // Inner loop
    static final double PAN_INNER_KP = 0.025;
    static final double PAN_INNER_KD = 0.0050;
    static final double PAN_INNER_KV = 0.0005;
    static final double PAN_INNER_KI = 0.005;
    static final double PAN_INNER_MAX_I = 0.05;
    double panInnerIntegral = 0;
    static final double PAN_INNER_STICTION = 0.030;
    static final double PAN_INNER_STICTION_FADE_TICKS = 5.0;
    static final double PAN_INNER_DEADBAND_TICKS = 1.0;

    static final double PAN_DIRECTION_SIGN_DEFAULT = 1.0;
    double panDirectionOverride = PAN_DIRECTION_SIGN_DEFAULT;
    boolean lastXButton = false;

    // ---------------- Latency compensation ----------------
    static final boolean LATENCY_COMP_ENABLED = false;
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

    double filteredPanMotorVelocity = 0;
    static final double VELOCITY_EMA_ALPHA = 0.3;

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
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);

        robot = Hardware.getInstance(hardwareMap);
        turret = robot.turret;
        limelight = robot.limelight;
        limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
        limelight.start();

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        openLogFile();

        telemetry.addLine("=== TURRET TRACKING READY ===");
        telemetry.addData("LOG", logWriter != null ? logPath : ("FAILED - " + logError));
        telemetry.addLine("X: flip pan direction | A: toggle lead prediction | dpad L/R: lead scale");
        telemetry.update();
        waitForStart();

        logTimer.reset();
        loopTimer.reset();
        telemetryTimer.reset();
        try {
            while (opModeIsActive()) {
                for (LynxModule hub : allHubs) {
                    hub.clearBulkCache();
                }

                // Mecanum drive using gamepad1 sticks
                double driveY = -gamepad1.left_stick_y;
                double driveX = gamepad1.left_stick_x;
                double driveRx = gamepad1.right_stick_x;

                double denominator = Math.max(Math.abs(driveY) + Math.abs(driveX) + Math.abs(driveRx), 1);
                double frontLeftPower = (driveY + driveX + driveRx) / denominator;
                double backLeftPower = (driveY - driveX + driveRx) / denominator;
                double frontRightPower = (driveY - driveX - driveRx) / denominator;
                double backRightPower = (driveY + driveX - driveRx) / denominator;

                robot.lf.setPower(frontLeftPower);
                robot.lb.setPower(backLeftPower);
                robot.rf.setPower(frontRightPower);
                robot.rb.setPower(backRightPower);

                double dtS = loopTimer.seconds();
                loopTimer.reset();
                double dtMs = dtS * 1000.0;
                double nowMs = logTimer.milliseconds();

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
                }
                lastAButton = gamepad1.a;

                if (gamepad1.dpad_left && !lastDpadLeft) {
                    leadScale = Math.max(LEAD_SCALE_MIN, leadScale - LEAD_SCALE_STEP);
                }
                if (gamepad1.dpad_right && !lastDpadRight) {
                    leadScale = Math.min(LEAD_SCALE_MAX, leadScale + LEAD_SCALE_STEP);
                }
                lastDpadLeft = gamepad1.dpad_left;
                lastDpadRight = gamepad1.dpad_right;

                LLResult result = limelight.getLatestResult();
                boolean fresh = (result != null && result.isValid() && (result.getCaptureLatency() + result.getTargetingLatency()) <= MAX_RESULT_STALENESS_MS);

                double frameTimestamp = (result != null) ? result.getTimestamp() : -1.0;
                boolean isNewFrame = fresh && (frameTimestamp != lastLimelightTimestamp);
                if (isNewFrame) {
                    lastLimelightTimestamp = frameTimestamp;
                }

                // ---------------- CLUSTER DETECTION & AGGREGATION ----------------
                boolean targetFound = false;
                double sumTx = 0;
                double sumTy = 0;
                double sumDist = 0;
                int validTagCount = 0;

                if (fresh) {
                    List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                    if (tags != null) {
                        for (LLResultTypes.FiducialResult tag : tags) {
                            boolean match = (TARGET_TAG_IDS.length == 0 || TARGET_TAG_IDS[0] < 0);
                            if (!match) {
                                for (int id : TARGET_TAG_IDS) {
                                    if (tag.getFiducialId() == id) {
                                        match = true;
                                        break;
                                    }
                                }
                            }
                            if (match) {
                                sumTx += tag.getTargetXDegrees();
                                sumTy += tag.getTargetYDegrees();

                                double totalAngleDeg = CAMERA_TILT_DEG + tag.getTargetYDegrees();
                                double heightDiff = TURRET_CAMERA_HEIGHT_IN - TARGET_TAG_HEIGHT_IN;
                                if (totalAngleDeg > MIN_VALID_DEPRESSION_ANGLE_DEG) {
                                    double rawDistanceIn = heightDiff / Math.tan(Math.toRadians(totalAngleDeg));
                                    if (rawDistanceIn > 0 && rawDistanceIn < 200.0) {
                                        sumDist += rawDistanceIn;
                                    }
                                }
                                validTagCount++;
                                targetFound = true;
                            }
                        }
                    }
                }

                double rawTx = 0;
                double tx = 0;
                double distanceIn = 36.0;
                double appliedPanPower = 0;

                int currentPanTicks = turret.getCurrentPosition();
                double rawVelocity = turret.getVelocity();
                filteredPanMotorVelocity = VELOCITY_EMA_ALPHA * rawVelocity + (1.0 - VELOCITY_EMA_ALPHA) * filteredPanMotorVelocity;
                double panMotorVelocity = filteredPanMotorVelocity;

                panTickHistory.addLast(new double[]{nowMs, currentPanTicks});
                while (!panTickHistory.isEmpty()
                        && nowMs - panTickHistory.peekFirst()[0] > PAN_HISTORY_WINDOW_MS) {
                    panTickHistory.removeFirst();
                }

                if (targetFound) {
                    consecutiveMissedFrames = 0;
                    consecutiveGoodFrames++;

                    rawTx = sumTx / validTagCount;
                    tx = medianFilterTx(rawTx);

                    double clusterTy = sumTy / validTagCount;
                    double totalAngleDeg = CAMERA_TILT_DEG + clusterTy;
                    double heightDiff = TURRET_CAMERA_HEIGHT_IN - TARGET_TAG_HEIGHT_IN;
                    double rawDistanceIn = distanceIn;
                    if (totalAngleDeg > MIN_VALID_DEPRESSION_ANGLE_DEG) {
                        rawDistanceIn = heightDiff / Math.tan(Math.toRadians(totalAngleDeg));
                    }
                    if (validTagCount > 0 && sumDist > 0) {
                        rawDistanceIn = sumDist / validTagCount;
                    }

                    if (!distanceFilterInitialized) {
                        filteredDistanceIn = rawDistanceIn;
                        distanceFilterInitialized = true;
                    } else {
                        filteredDistanceIn = DISTANCE_EMA_ALPHA * rawDistanceIn
                                + (1.0 - DISTANCE_EMA_ALPHA) * filteredDistanceIn;
                    }
                    distanceIn = filteredDistanceIn;

                    // Range rate estimation
                    if (!rangeRateInitialized) {
                        lastDistanceForRate = distanceIn;
                        lastDistanceTimeMs = nowMs;
                        rangeRateInitialized = true;
                    } else {
                        double distDtMs = nowMs - lastDistanceTimeMs;
                        if (distDtMs >= 40.0) {
                            double instantRate = (distanceIn - lastDistanceForRate) / (distDtMs / 1000.0);
                            rangeRateInPerSec = RANGE_RATE_SMOOTH * instantRate
                                    + (1.0 - RANGE_RATE_SMOOTH) * rangeRateInPerSec;
                            lastDistanceForRate = distanceIn;
                            lastDistanceTimeMs = nowMs;
                        }
                    }
                    rangeRateInPerSec = clamp(rangeRateInPerSec, -MAX_RANGE_RATE_IN_PER_SEC, MAX_RANGE_RATE_IN_PER_SEC);

                    impactDistanceIn = distanceIn;
                    tofS = TOF_INTERCEPT_S + TOF_PER_INCH_S * impactDistanceIn;
                    timeToImpactS = clamp(tofS + MECH_LATENCY_S, 0.0, MAX_LEAD_TIME_S);

                    // Latency compensation
                    double latencyMs = result.getCaptureLatency() + result.getTargetingLatency() + PAN_LATENCY_FUDGE_MS;
                    lastLatencyMs = latencyMs;

                    double turretMovedDeg = 0;
                    if (LATENCY_COMP_ENABLED && !panTickHistory.isEmpty()) {
                        double targetHistoryTimeMs = nowMs - latencyMs;
                        double[] pastEntry = panTickHistory.peekFirst();
                        for (double[] entry : panTickHistory) {
                            if (entry[0] <= targetHistoryTimeMs) {
                                pastEntry = entry;
                            } else {
                                break;
                            }
                        }
                        if (pastEntry != null) {
                            int ticksAtCapture = (int) pastEntry[1];
                            turretMovedDeg = (currentPanTicks - ticksAtCapture) * PAN_DEGREES_PER_TICK;
                            lastTurretMovedDeg = turretMovedDeg;
                        }
                    }

                    double adjustedTx = tx + turretMovedDeg * PAN_LATENCY_SIGN;

                    if (!bearingInitialized) {
                        lastBearingDeg = adjustedTx;
                        lastBearingTimeMs = nowMs;
                        bearingInitialized = true;
                        targetRateDegPerSec = 0;
                    } else {
                        double bearingDtMs = nowMs - lastBearingTimeMs;
                        if (bearingDtMs >= 15.0) {
                            if (bearingDtMs > RATE_MAX_GAP_MS) {
                                bearingInitialized = false;
                                targetRateDegPerSec = 0;
                            } else {
                                double rawRate = (adjustedTx - lastBearingDeg) / (bearingDtMs / 1000.0);
                                rawRate = clamp(rawRate, -PAN_MAX_TARGET_RATE_DPS, PAN_MAX_TARGET_RATE_DPS);
                                targetRateDegPerSec = PAN_TARGET_RATE_SMOOTH * rawRate
                                        + (1.0 - PAN_TARGET_RATE_SMOOTH) * targetRateDegPerSec;
                                lastBearingDeg = adjustedTx;
                                lastBearingTimeMs = nowMs;
                            }
                        }
                    }

                    double effectiveRate = 0.0;
                    if (predictionEnabled && Math.abs(targetRateDegPerSec) > PAN_RATE_DEADBAND_DPS) {
                        effectiveRate = targetRateDegPerSec
                                - Math.copySign(PAN_RATE_DEADBAND_DPS, targetRateDegPerSec);
                    }

                    panLeadDeg = clamp(effectiveRate * timeToImpactS * leadScale,
                            -MAX_LEAD_DEG, MAX_LEAD_DEG);

                    double panErrorDeg = (adjustedTx - PAN_AIM_OFFSET_DEG) + panLeadDeg;
                    aimReady = Math.abs(panErrorDeg) <= PAN_DEADBAND_DEG;

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

                } else {
                    consecutiveMissedFrames++;
                    consecutiveGoodFrames = 0;

                    if (consecutiveMissedFrames > PAN_COAST_FRAMES) {
                        targetRateDegPerSec *= PAN_COAST_DECAY;
                    }
                    aimReady = false;
                    panLeadDeg = 0;

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

                    if (Math.abs(appliedPanPower) < PAN_MAX_POWER * 0.95) {
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

                    // Scale power down when close to target for smooth micro-adjustments
                    double proximityScale = clamp(Math.abs(tickError) / 25.0, 0.25, 1.0);
                    innerPower *= proximityScale;

                    innerPower *= PAN_TICK_SIGN;
                }

                appliedPanPower = clamp(innerPower, -PAN_MAX_POWER, PAN_MAX_POWER);

                double tickDirection = PAN_TICK_SIGN * appliedPanPower;
                if (currentPanTicks >= PAN_MAX_TICKS && tickDirection > 0) appliedPanPower = 0;
                if (currentPanTicks <= PAN_MIN_TICKS && tickDirection < 0) appliedPanPower = 0;
                turret.setPower(appliedPanPower);

                String csvLine = String.format(Locale.US,
                        "%.0f,%.1f,%d,%+.2f,%+.2f,%+.2f,%+.0f,%+.1f,%.0f,%+.2f,%d,%.1f,%+.2f,%.1f,%+.1f,%.1f,%+.2f,%.3f",
                        nowMs, dtMs, (targetFound ? 1 : 0), rawTx, tx, appliedPanPower,
                        panMotorVelocity, targetRateDegPerSec, lastLatencyMs, lastTurretMovedDeg,
                        currentPanTicks, panSetpointTicks, lastTickError,
                        distanceIn, rangeRateInPerSec, impactDistanceIn,
                        panLeadDeg, timeToImpactS);

                logDisk(csvLine);

                if (telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS) {
                    telemetryTimer.reset();

                    liveCsvLines.addLast(csvLine);
                    while (liveCsvLines.size() > SCREEN_CSV_LINES) {
                        liveCsvLines.removeFirst();
                    }

                    telemetry.addLine(aimReady ? ">>> AIM READY <<<" : "... tracking ...");
                    telemetry.addData("Cascade", "setpt %.0f / actual %d / err %+.1f tk",
                            panSetpointTicks, currentPanTicks, lastTickError);
                    telemetry.addData("Vision", "%s  miss=%d  lat=%.0f ms",
                            targetFound ? "TRACKING" : "NO TARGET",
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
                    telemetry.addData("Dist", "now %.1f -> impact %.1f in",
                            distanceIn, impactDistanceIn);
                    telemetry.addData("Target closing", "%+.1f in/s", rangeRateInPerSec);
                    telemetry.addData("Sweep", "%+.1f deg/s -> aim ahead %+.2f deg",
                            targetRateDegPerSec, panLeadDeg);
                    telemetry.addData("Latency", "%.0f ms -> %+.2f deg lead",
                            lastLatencyMs, lastTurretMovedDeg);
                    telemetry.addData("Loop", "%.1f ms", dtMs);
                    telemetry.addData("Log", logWriter != null ? logPath : ("FAILED " + logError));
                    telemetry.addLine("t,dt,ok,tx,flt,pwr,pvel,trate,lat,mvd,tick,setpt,terr,dist,rrate,idist,plead,T");
                    for (String line : liveCsvLines) {
                        telemetry.addLine(line);
                    }
                    telemetry.update();
                }
            }
        } finally {
            closeLogFile();
        }
    }

    private double medianFilterTx(double newTx) {
        txHistory[txHistoryCount % txHistory.length] = newTx;
        txHistoryCount++;
        int count = Math.min(txHistoryCount, txHistory.length);
        double[] temp = new double[count];
        System.arraycopy(txHistory, 0, temp, 0, count);
        Arrays.sort(temp);
        return temp[count / 2];
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void openLogFile() {
        try {
            File dir = new File(LOG_DIR);
            if (!dir.exists()) {
                if (!dir.mkdirs()) {
                    logError = "mkdirs failed: " + dir.getPath();
                    return;
                }
            }
            logFileName = LOG_PREFIX + new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date()) + ".csv";
            logPath = new File(dir, logFileName).getPath();
            logWriter = new BufferedWriter(new FileWriter(logPath));
            logWriter.write("time_ms,dt_ms,ok,tx,flt,pwr,pvel,trate,lat,mvd,tick,setpt,terr,dist,rrate,idist,plead,T\n");
            logError = "";
        } catch (Exception e) {
            logError = e.getClass().getSimpleName() + ": " + e.getMessage();
        }
    }

    private void logDisk(String line) {
        if (logWriter == null) return;
        try {
            logWriter.write(line + "\n");
            logLoopCounter++;
            if (logLoopCounter >= LOG_FLUSH_EVERY_N_LOOPS) {
                logWriter.flush();
                logLoopCounter = 0;
            }
        } catch (Exception e) {
            logError = e.getClass().getSimpleName() + ": " + e.getMessage();
        }
    }

    private void closeLogFile() {
        if (logWriter != null) {
            try {
                logWriter.flush();
                logWriter.close();
            } catch (IOException e) {
                logError = e.getClass().getSimpleName() + ": " + e.getMessage();
            }
        }
    }
}
