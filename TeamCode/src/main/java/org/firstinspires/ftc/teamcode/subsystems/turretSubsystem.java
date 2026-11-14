package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Deque;
import java.util.List;

/** Owns AprilTag tracking, motion prediction, and turret motor control (Merged & Optimized). */
public class turretSubsystem {

    // ========================= VALUES TO CALIBRATE =========================

    /** Displayed by the test TeleOp so the deployed tuning profile is verifiable. */
    public static final String TUNE_VERSION = "competition-v1-cluster";

    /** Limelight AprilTag pipeline number. */
    public static final int APRILTAG_PIPELINE = 1;

    public enum Alliance { RED, BLUE }

    /** First AprilTag ID in each four-tag CELL cluster. */
    public static final int RED_SCORING_CLUSTER_FIRST_ID = 30;
    public static final int RED_AUDIENCE_CLUSTER_FIRST_ID = 34;
    public static final int BLUE_AUDIENCE_CLUSTER_FIRST_ID = 38;
    public static final int BLUE_SCORING_CLUSTER_FIRST_ID = 42;

    /** Per-tag optical corrections, in the same order. Positive subtracts from measured Tx. */
    public static final double[] TAG_AIM_OFFSETS_DEG = {};

    /** Optional per-tag heights, in the same order. Empty uses DEFAULT_TAG_HEIGHT_IN. */
    public static final double[] TAG_HEIGHTS_IN = {};

    /** Encoder ticks per degree (System 1 calibrated value). */
    public static final double TICKS_PER_DEGREE = 1.78;

    /** Flip this sign if the turret corrects away from the target. */
    public static final double AIM_DIRECTION_SIGN = 1.0;

    /** Flip this sign if motor power and encoder tick direction disagree. */
    public static final double MOTOR_DIRECTION_SIGN = 1.0;

    /** Encoder safety limits (System 1 limits). */
    public static final int MIN_TURRET_TICKS = -140;
    public static final int MAX_TURRET_TICKS = 140;
    public static final double MAX_TURRET_POWER = 0.70;

    /** Camera geometry for calculating horizontal floor distance. */
    public static final double CAMERA_HEIGHT_IN = 48.0;
    public static final double DEFAULT_TAG_HEIGHT_IN = 14.5;
    public static final double CAMERA_DOWN_ANGLE_DEG = 24.24;

    /**
     * Official SDK 12 BIOBUZZ cluster geometry for IDs 42-45. Tag X positions
     * are relative to the CELL opening center, which is the official aim origin.
     */
    public static final double[] CLUSTER_TAG_X_OFFSETS_IN = {
            -6.50, -2.75, 2.75, 6.50
    };
    public static final double CLUSTER_AIM_POINT_FORWARD_OF_TAGS_IN = 7.1874;
    /** Official SDK Z offset: tag centers are this far below the opening origin. */
    public static final double CLUSTER_AIM_POINT_ABOVE_TAGS_IN = 5.622;

    /** Conservative smooth baseline, used with velocity and proximity smoothing below. */
    public static final double POSITION_KP = 0.045;
    public static final double VELOCITY_KD = 0.0045;
    public static final double VELOCITY_KV = 0.00150;
    public static final double POSITION_KI = 0.005;
    public static final double MAX_INTEGRAL_POWER = 0.025;
    public static final double STICTION_POWER = 0.30;

    /** Lower values trade reaction speed for less vision-induced twitching. */
    public static final double VISION_CORRECTION_GAIN = 0.160;
    public static final double AIM_TOLERANCE_DEG = 0.40;
    public static final int AIM_STABLE_FRAMES = 2;

    /** System 1 leaves latency compensation off for smoother initial tuning. */
    public static boolean LATENCY_COMPENSATION_ENABLED = false;

    /** Motion prediction: tune flight time after basic stationary aiming works. */
    public static boolean PREDICTION_ENABLED = true;
    public static double FIXED_FLIGHT_TIME_S = 0.12;
    public static double TOF_PER_INCH_S = 0.0055;
    public static double FIXED_SHOT_DELAY_S = 0.06;
    public static double MAX_PREDICTION_TIME_S = 1.20;
    public static double MAX_ANGLE_LEAD_DEG = 12.0;
    public static double MAX_DISTANCE_LEAD_IN = 36.0;

    /** Odometry prediction settings. */
    public static boolean USE_ODOMETRY_PREDICTION = true;
    public static double ODOMETRY_RATE_BLEND = 0.70;
    public static double ODOMETRY_FORWARD_SIGN = 1.0;
    public static double ODOMETRY_LEFT_SIGN = 1.0;
    public static double ODOMETRY_HEADING_SIGN = 1.0;
    public static double ODOMETRY_BEARING_SIGN = -1.0;
    /** Maximum dead-reckoning time after vision loss; BIOBUZZ CELLs can move. */
    public static double ODOMETRY_TARGET_HOLD_MS = 1000.0;

    // ========================= NORMAL CONSTANTS =========================

    private static final long MAX_RESULT_AGE_MS = 120;
    private static final long HOLD_AFTER_TARGET_LOSS_MS = 350;
    private static final double MIN_DISTANCE_ANGLE_DEG = 2.0;
    private static final double MAX_DISTANCE_IN = 200.0;
    private static final double DISTANCE_FILTER_ALPHA = 0.25;
    private static final double TARGET_RATE_FILTER_ALPHA = 0.10;
    private static final double RANGE_RATE_FILTER_ALPHA = 0.25;
    private static final double RATE_DEADBAND_DEG_PER_SEC = 4.0;
    private static final double MAX_TARGET_RATE_DEG_PER_SEC = 60.0;
    private static final double MAX_RANGE_RATE_IN_PER_SEC = 140.0;
    private static final double MAX_RATE_SAMPLE_GAP_MS = 180.0;
    private static final double MIN_INTERCEPT_DENOMINATOR = 0.45;
    private static final double EXTRA_LATENCY_MS = 10.0;
    private static final double ENCODER_HISTORY_MS = 400.0;
    private static final double SETPOINT_DEADBAND_TICKS = 0.8;
    private static final double STICTION_FADE_TICKS = 5.0;
    private static final double MOTOR_VELOCITY_FILTER_ALPHA = 0.30;
    private static final double FULL_POWER_ERROR_TICKS = 25.0;
    private static final double MIN_PROXIMITY_POWER_SCALE = 0.25;

    private final Hardware robot;
    private final DcMotorEx turretMotor;
    private final ElapsedTime clock = new ElapsedTime();
    private final Deque<double[]> encoderHistory = new ArrayDeque<>();
    private final double[] txHistory = new double[5];
    private final double[] distanceHistory = new double[5];

    private boolean autoAimEnabled;
    private double manualPower;
    private int selectedTagId = -1;
    private Alliance alliance = Alliance.BLUE;
    private int selectedClusterFirstId = -1;
    private double selectedClusterRollDeg = Double.NaN;
    private String visibleTagIds = "none";
    private int visibleTagCount;
    private boolean clusterGeometryValid;
    private double lastClusterTagPlaneDepthIn = Double.NaN;
    private double lastFrameTimestamp = -1;
    private double lastTargetSeenMs = -1;

    private int txHistoryCount;
    private int distanceHistoryCount;
    private double filteredTxDeg;
    private double filteredDistanceIn;
    private boolean distanceInitialized;

    private double lastBearingDeg;
    private double lastBearingTimeMs = -1;
    private boolean bearingInitialized;
    private double targetRateDegPerSec;
    private double lastDistanceIn;
    private double lastDistanceTimeMs = -1;
    private boolean rangeRateInitialized;
    private double rangeRateInPerSec;

    private double robotForwardVelocityInPerSec;
    private double robotLeftVelocityInPerSec;
    private double robotHeadingRateRadPerSec;
    private double robotFieldXIn;
    private double robotFieldYIn;
    private double robotFieldHeadingRad;
    private double odometryTargetRateDegPerSec;
    private double odometryRangeRateInPerSec;
    private double lastRobotBearingDeg = Double.NaN;
    private boolean odometryRateValid;
    private boolean odometryPoseValid;

    /** Last vision target stored in Pinpoint field coordinates. */
    private double odometryTargetFieldXIn;
    private double odometryTargetFieldYIn;
    private double odometryTrackedBearingDeg;
    private double odometryTrackedDistanceIn;
    private boolean odometryTargetInitialized;
    private boolean odometryTrackingActive;

    private boolean targetVisible;
    private boolean aimReady;
    private int stableAimFrames;
    private double panLeadDeg;
    private double impactDistanceIn;
    private double predictionTimeS;
    private double lastVisionErrorDeg;
    private double lastLatencyMs;

    private double targetTicks;
    private double targetVelocityTicksPerSecond;
    private boolean targetTicksInitialized;
    private double integralPower;
    private double appliedPower;
    private double filteredMotorVelocityTicksPerSecond;
    private boolean motorVelocityInitialized;

    public turretSubsystem(Hardware robot) {
        this.robot = robot;
        turretMotor = robot.turret;
        // Never carry a stale command into this subsystem's initialization.
        turretMotor.setPower(0);
        robot.limelight.pipelineSwitch(APRILTAG_PIPELINE);
        robot.limelight.setPollRateHz(100);
        robot.limelight.start();
        clock.reset();
    }

    public void setAutoAimEnabled(boolean enabled) {
        if (autoAimEnabled == enabled) return;
        autoAimEnabled = enabled;
        targetTicksInitialized = false;
        integralPower = 0;
        stableAimFrames = 0;
        aimReady = false;
        resetMeasurementHistory();
    }

    public void setManualPower(double power) {
        manualPower = clamp(power, -MAX_TURRET_POWER, MAX_TURRET_POWER);
    }

    public void setAlliance(Alliance alliance) {
        if (alliance == null || this.alliance == alliance) return;
        this.alliance = alliance;
        selectedClusterFirstId = -1;
        selectedClusterRollDeg = Double.NaN;
        selectedTagId = -1;
        targetVisible = false;
        targetTicksInitialized = false;
        resetMeasurementHistory();
    }

    /**
     * Convenience entry point for adding the turret to another TeleOp. This
     * also updates Pinpoint odometry and Limelight tracking internally.
     */
    public void update(double dtSeconds, boolean autoAimEnabled, double manualPower) {
        setAutoAimEnabled(autoAimEnabled);
        setManualPower(manualPower);
        update(dtSeconds);
    }

    /**
     * Defines the current physical angle as the center of the software limits.
     * Call only when the turret is at its intended forward/center position.
     */
    public void zeroAtCurrentPosition() {
        turretMotor.setPower(0);
        robot.setTurretCurrentPositionAsZero();
        targetTicks = 0;
        targetVelocityTicksPerSecond = 0;
        targetTicksInitialized = false;
        integralPower = 0;
        appliedPower = 0;
        manualPower = 0;
        encoderHistory.clear();
        motorVelocityInitialized = false;
        stableAimFrames = 0;
        aimReady = false;
    }

    /** Call once per OpMode loop. */
    public void update(double dtSeconds) {
        double dt = clamp(dtSeconds, 0.001, 0.100);
        double nowMs = clock.milliseconds();
        int currentTicks = robot.getTurretPositionTicks();
        updateFilteredMotorVelocity();

        updateOdometry();
        saveEncoderSample(nowMs, currentTicks);
        updateVision(nowMs, currentTicks);
        updateLostTargetFromOdometry(nowMs, currentTicks);
        updateOdometryPredictionRates();

        if (autoAimEnabled) {
            applyAutomaticControl(dt, nowMs, currentTicks);
        } else {
            applyManualControl(currentTicks);
        }
    }

    private void updateVision(double nowMs, int currentTicks) {
        LLResult result = robot.limelight.getLatestResult();
        boolean fresh = result != null && result.isValid()
                && result.getStaleness() >= 0
                && result.getStaleness() < MAX_RESULT_AGE_MS;

        List<LLResultTypes.FiducialResult> tags = fresh ? result.getFiducialResults() : null;
        updateVisibleTagText(tags);
        List<LLResultTypes.FiducialResult> clusterTags = selectClusterTags(tags);
        targetVisible = !clusterTags.isEmpty();
        visibleTagCount = clusterTags.size();

        if (!targetVisible) {
            clusterGeometryValid = false;
            aimReady = false;
            stableAimFrames = 0;
            targetRateDegPerSec *= 0.90;
            panLeadDeg = 0;
            return;
        }

        odometryTrackingActive = false;

        double frameTimestamp = result.getTimestamp();
        if (frameTimestamp == lastFrameTimestamp) return;
        lastFrameTimestamp = frameTimestamp;
        lastTargetSeenMs = nowMs;

        ClusterAim clusterAim = calculateClusterAim(clusterTags);
        filteredTxDeg = addAndMedian(txHistory, txHistoryCount, clusterAim.txDeg);
        txHistoryCount = Math.min(txHistoryCount + 1, txHistory.length);
        updateDistance(clusterAim.averageTyDeg, selectedTagId,
                clusterAim.horizontalDistanceIn);

        lastLatencyMs = result.getStaleness() + result.getCaptureLatency()
                + result.getTargetingLatency() + EXTRA_LATENCY_MS;
        double captureTimeMs = nowMs - lastLatencyMs;
        double ticksAtCapture = encoderTicksAt(captureTimeMs);
        // A positive turret movement reduces a same-direction image error.
        double turretMovementDeg = LATENCY_COMPENSATION_ENABLED
                ? -(currentTicks - ticksAtCapture)
                        / (AIM_DIRECTION_SIGN * TICKS_PER_DEGREE) : 0;
        double horizontalErrorDeg = filteredTxDeg;

        // Robot-relative bearing = camera error + physical turret bearing.
        double bearingAtCaptureDeg = horizontalErrorDeg
                + ticksAtCapture / (AIM_DIRECTION_SIGN * TICKS_PER_DEGREE);
        lastRobotBearingDeg = bearingAtCaptureDeg;
        updateTargetRates(captureTimeMs, bearingAtCaptureDeg);
        saveTargetInFieldCoordinates(
                bearingAtCaptureDeg, filteredDistanceIn, lastLatencyMs);
        updatePrediction(lastLatencyMs);

        lastVisionErrorDeg = horizontalErrorDeg + turretMovementDeg + panLeadDeg;
        stableAimFrames = Math.abs(lastVisionErrorDeg) <= AIM_TOLERANCE_DEG
                ? stableAimFrames + 1 : 0;
        aimReady = stableAimFrames >= AIM_STABLE_FRAMES;

        if (autoAimEnabled) {
            double measuredTargetTicks = currentTicks
                    + AIM_DIRECTION_SIGN * lastVisionErrorDeg * TICKS_PER_DEGREE;
            if (!targetTicksInitialized) {
                targetTicks = measuredTargetTicks;
                targetTicksInitialized = true;
            } else {
                targetTicks = (1.0 - VISION_CORRECTION_GAIN) * targetTicks
                        + VISION_CORRECTION_GAIN * measuredTargetTicks;
            }
        }
    }

    private void updateDistance(
            double targetYDegrees, int tagId, double clusterDistanceIn) {
        double rawDistanceIn = clusterDistanceIn;
        if (!Double.isFinite(rawDistanceIn)) {
            double depressionAngleDeg = CAMERA_DOWN_ANGLE_DEG - targetYDegrees;
            if (Math.abs(depressionAngleDeg) <= MIN_DISTANCE_ANGLE_DEG) return;
            rawDistanceIn = (CAMERA_HEIGHT_IN - tagHeightForId(tagId))
                    / Math.tan(Math.toRadians(depressionAngleDeg));
        }
        if (rawDistanceIn <= 0 || rawDistanceIn >= MAX_DISTANCE_IN) return;

        double medianDistance = addAndMedian(
                distanceHistory, distanceHistoryCount, rawDistanceIn);
        distanceHistoryCount = Math.min(distanceHistoryCount + 1, distanceHistory.length);

        if (!distanceInitialized) {
            filteredDistanceIn = medianDistance;
            impactDistanceIn = medianDistance;
            distanceInitialized = true;
        } else {
            filteredDistanceIn += DISTANCE_FILTER_ALPHA
                    * (medianDistance - filteredDistanceIn);
        }
    }

    private void updateTargetRates(double captureTimeMs, double bearingDeg) {
        if (bearingInitialized) {
            double elapsedMs = captureTimeMs - lastBearingTimeMs;
            if (elapsedMs > 1 && elapsedMs <= MAX_RATE_SAMPLE_GAP_MS) {
                double measuredRate = clamp(
                        (bearingDeg - lastBearingDeg) / (elapsedMs / 1000.0),
                        -MAX_TARGET_RATE_DEG_PER_SEC, MAX_TARGET_RATE_DEG_PER_SEC);
                if (Math.abs(measuredRate) < RATE_DEADBAND_DEG_PER_SEC) measuredRate = 0;
                targetRateDegPerSec += TARGET_RATE_FILTER_ALPHA
                        * (measuredRate - targetRateDegPerSec);
            } else {
                targetRateDegPerSec = 0;
            }
        }
        lastBearingDeg = bearingDeg;
        lastBearingTimeMs = captureTimeMs;
        bearingInitialized = true;

        if (!distanceInitialized) return;
        if (rangeRateInitialized) {
            double elapsedMs = captureTimeMs - lastDistanceTimeMs;
            if (elapsedMs > 1 && elapsedMs <= MAX_RATE_SAMPLE_GAP_MS) {
                double measuredRate = clamp(
                        (filteredDistanceIn - lastDistanceIn) / (elapsedMs / 1000.0),
                        -MAX_RANGE_RATE_IN_PER_SEC, MAX_RANGE_RATE_IN_PER_SEC);
                rangeRateInPerSec += RANGE_RATE_FILTER_ALPHA
                        * (measuredRate - rangeRateInPerSec);
            } else {
                rangeRateInPerSec = 0;
            }
        }
        lastDistanceIn = filteredDistanceIn;
        lastDistanceTimeMs = captureTimeMs;
        rangeRateInitialized = true;
    }

    /** Reads and owns the odometry state used by turret prediction/tracking. */
    private void updateOdometry() {
        if (robot.odo == null) return;
        robot.odo.update();

        double forwardVelocity = robot.odo.getVelX(DistanceUnit.INCH);
        double leftVelocity = robot.odo.getVelY(DistanceUnit.INCH);
        double headingRateDegPerSec = robot.odo.getHeadingVelocity(
                UnnormalizedAngleUnit.DEGREES);
        double fieldX = robot.odo.getPosX(DistanceUnit.INCH);
        double fieldY = robot.odo.getPosY(DistanceUnit.INCH);
        double headingRad = robot.odo.getHeading(AngleUnit.RADIANS);

        odometryRateValid = Double.isFinite(forwardVelocity)
                && Double.isFinite(leftVelocity)
                && Double.isFinite(headingRateDegPerSec);
        odometryPoseValid = Double.isFinite(fieldX)
                && Double.isFinite(fieldY)
                && Double.isFinite(headingRad);
        if (odometryPoseValid) {
            robotFieldXIn = fieldX;
            robotFieldYIn = fieldY;
            robotFieldHeadingRad = headingRad;
        }
        if (!odometryRateValid) return;

        // Pinpoint X/Y velocity is already robot-relative: X forward, Y left.
        robotForwardVelocityInPerSec = ODOMETRY_FORWARD_SIGN * forwardVelocity;
        robotLeftVelocityInPerSec = ODOMETRY_LEFT_SIGN * leftVelocity;
        robotHeadingRateRadPerSec = ODOMETRY_HEADING_SIGN
                * Math.toRadians(headingRateDegPerSec);
    }

    /** Converts the latest camera-relative target into Pinpoint field coordinates. */
    private void saveTargetInFieldCoordinates(
            double bearingAtCaptureDeg, double distanceAtCaptureIn, double latencyMs) {
        if (!odometryPoseValid || !Double.isFinite(distanceAtCaptureIn)
                || distanceAtCaptureIn < 1.0) {
            return;
        }

        double mathBearingRad = Math.toRadians(
                ODOMETRY_BEARING_SIGN * bearingAtCaptureDeg);
        double relativeForwardIn = distanceAtCaptureIn * Math.cos(mathBearingRad);
        double relativeLeftIn = distanceAtCaptureIn * Math.sin(mathBearingRad);

        // Advance the capture-time measurement to approximately "now" using
        // the current robot twist before anchoring it in the field frame.
        if (odometryRateValid) {
            double latencyS = clamp(latencyMs / 1000.0, 0, 0.25);
            relativeForwardIn -= robotForwardVelocityInPerSec * latencyS;
            relativeLeftIn -= robotLeftVelocityInPerSec * latencyS;
            double headingChange = robotHeadingRateRadPerSec * latencyS;
            double cosChange = Math.cos(headingChange);
            double sinChange = Math.sin(headingChange);
            double rotatedForward = cosChange * relativeForwardIn
                    + sinChange * relativeLeftIn;
            double rotatedLeft = -sinChange * relativeForwardIn
                    + cosChange * relativeLeftIn;
            relativeForwardIn = rotatedForward;
            relativeLeftIn = rotatedLeft;
        }

        double cosHeading = Math.cos(robotFieldHeadingRad);
        double sinHeading = Math.sin(robotFieldHeadingRad);
        odometryTargetFieldXIn = robotFieldXIn
                + relativeForwardIn * cosHeading - relativeLeftIn * sinHeading;
        odometryTargetFieldYIn = robotFieldYIn
                + relativeForwardIn * sinHeading + relativeLeftIn * cosHeading;
        odometryTargetInitialized = true;
    }

    /** Uses the saved field point as the aim source while vision is occluded. */
    private void updateLostTargetFromOdometry(double nowMs, int currentTicks) {
        odometryTrackingActive = false;
        if (targetVisible) return;

        double targetAgeMs = lastTargetSeenMs < 0
                ? Double.POSITIVE_INFINITY : nowMs - lastTargetSeenMs;
        boolean canTrack = USE_ODOMETRY_PREDICTION
                && odometryTargetInitialized
                && odometryPoseValid
                && targetAgeMs <= ODOMETRY_TARGET_HOLD_MS;

        if (!canTrack) {
            if (targetAgeMs > HOLD_AFTER_TARGET_LOSS_MS) {
                selectedTagId = -1;
                resetMeasurementHistory();
            }
            return;
        }

        double fieldDeltaX = odometryTargetFieldXIn - robotFieldXIn;
        double fieldDeltaY = odometryTargetFieldYIn - robotFieldYIn;
        double cosHeading = Math.cos(robotFieldHeadingRad);
        double sinHeading = Math.sin(robotFieldHeadingRad);
        double relativeForwardIn = fieldDeltaX * cosHeading
                + fieldDeltaY * sinHeading;
        double relativeLeftIn = -fieldDeltaX * sinHeading
                + fieldDeltaY * cosHeading;
        double distanceIn = Math.hypot(relativeForwardIn, relativeLeftIn);
        if (!Double.isFinite(distanceIn) || distanceIn < 1.0
                || distanceIn > MAX_DISTANCE_IN) {
            resetMeasurementHistory();
            return;
        }

        double mathBearingDeg = Math.toDegrees(
                Math.atan2(relativeLeftIn, relativeForwardIn));
        double bearingDeg = ODOMETRY_BEARING_SIGN * mathBearingDeg;
        odometryTrackedBearingDeg = bearingDeg;
        odometryTrackedDistanceIn = distanceIn;
        lastRobotBearingDeg = bearingDeg;
        filteredDistanceIn = distanceIn;
        impactDistanceIn = distanceIn;
        distanceInitialized = true;
        odometryTrackingActive = true;

        double physicalTurretBearingDeg = currentTicks
                / (AIM_DIRECTION_SIGN * TICKS_PER_DEGREE);
        lastVisionErrorDeg = bearingDeg - physicalTurretBearingDeg;
        if (autoAimEnabled) {
            targetTicks = AIM_DIRECTION_SIGN * bearingDeg * TICKS_PER_DEGREE;
            targetTicksInitialized = true;
        }
    }

    private void updateOdometryPredictionRates() {
        if (!odometryRateValid || !distanceInitialized
                || Double.isNaN(lastRobotBearingDeg)
                || filteredDistanceIn < 1.0) {
            return;
        }

        double bearingRad = Math.toRadians(
                ODOMETRY_BEARING_SIGN * lastRobotBearingDeg);
        double cosBearing = Math.cos(bearingRad);
        double sinBearing = Math.sin(bearingRad);

        double velocityTowardTarget = robotForwardVelocityInPerSec * cosBearing
                + robotLeftVelocityInPerSec * sinBearing;
        double velocityLeftOfTarget = -robotForwardVelocityInPerSec * sinBearing
                + robotLeftVelocityInPerSec * cosBearing;

        odometryRangeRateInPerSec = clamp(
                -velocityTowardTarget,
                -MAX_RANGE_RATE_IN_PER_SEC,
                MAX_RANGE_RATE_IN_PER_SEC);

        double mathBearingRateRadPerSec = -robotHeadingRateRadPerSec
                - velocityLeftOfTarget / filteredDistanceIn;
        odometryTargetRateDegPerSec = clamp(
                ODOMETRY_BEARING_SIGN * Math.toDegrees(mathBearingRateRadPerSec),
                -MAX_TARGET_RATE_DEG_PER_SEC,
                MAX_TARGET_RATE_DEG_PER_SEC);
    }

    private double getPredictionTargetRateDegPerSec() {
        if (!USE_ODOMETRY_PREDICTION || !odometryRateValid) {
            return targetRateDegPerSec;
        }
        return (1.0 - ODOMETRY_RATE_BLEND) * targetRateDegPerSec
                + ODOMETRY_RATE_BLEND * odometryTargetRateDegPerSec;
    }

    private double getPredictionRangeRateInPerSec() {
        if (!USE_ODOMETRY_PREDICTION || !odometryRateValid) {
            return rangeRateInPerSec;
        }
        return (1.0 - ODOMETRY_RATE_BLEND) * rangeRateInPerSec
                + ODOMETRY_RATE_BLEND * odometryRangeRateInPerSec;
    }

    private void updatePrediction(double latencyMs) {
        if (!distanceInitialized) return;
        if (!PREDICTION_ENABLED) {
            predictionTimeS = 0;
            impactDistanceIn = filteredDistanceIn;
            panLeadDeg = 0;
            return;
        }

        double launchDelayS = FIXED_SHOT_DELAY_S + latencyMs / 1000.0;
        double predictionRangeRate = getPredictionRangeRateInPerSec();
        double denominator = Math.max(
                1.0 - TOF_PER_INCH_S * predictionRangeRate,
                MIN_INTERCEPT_DENOMINATOR);
        predictionTimeS = clamp(
                (launchDelayS + FIXED_FLIGHT_TIME_S
                        + TOF_PER_INCH_S * filteredDistanceIn) / denominator,
                0, MAX_PREDICTION_TIME_S);

        impactDistanceIn = clamp(
                filteredDistanceIn + clamp(
                        predictionRangeRate * predictionTimeS,
                        -MAX_DISTANCE_LEAD_IN, MAX_DISTANCE_LEAD_IN),
                0, MAX_DISTANCE_IN);

        double usefulRate = removeRateDeadband(getPredictionTargetRateDegPerSec());
        panLeadDeg = clamp(
                usefulRate * predictionTimeS,
                -MAX_ANGLE_LEAD_DEG, MAX_ANGLE_LEAD_DEG);
    }

    private void applyAutomaticControl(double dt, double nowMs, int currentTicks) {
        if (!targetTicksInitialized) targetTicks = currentTicks;

        if (odometryTrackingActive) {
            targetVelocityTicksPerSecond = AIM_DIRECTION_SIGN
                    * odometryTargetRateDegPerSec * TICKS_PER_DEGREE;
        } else if (lastTargetSeenMs < 0
                || nowMs - lastTargetSeenMs > HOLD_AFTER_TARGET_LOSS_MS) {
            targetTicks = currentTicks;
            targetVelocityTicksPerSecond = 0;
            integralPower = 0;
        } else if (PREDICTION_ENABLED) {
            targetVelocityTicksPerSecond = AIM_DIRECTION_SIGN
                    * removeRateDeadband(getPredictionTargetRateDegPerSec())
                    * TICKS_PER_DEGREE;
        } else {
            // Match System 1: prediction off means a stationary encoder setpoint.
            targetVelocityTicksPerSecond = 0;
        }

        targetTicks = clamp(targetTicks, MIN_TURRET_TICKS, MAX_TURRET_TICKS);
        double positionError = targetTicks - currentTicks;
        double velocityError = targetVelocityTicksPerSecond
                - filteredMotorVelocityTicksPerSecond;

        if (Math.abs(positionError) < SETPOINT_DEADBAND_TICKS
                && Math.abs(targetVelocityTicksPerSecond) < 2.0) {
            appliedPower = 0;
            integralPower *= 0.90;
        } else {
            integralPower = clamp(
                    integralPower + POSITION_KI * positionError * dt,
                    -MAX_INTEGRAL_POWER, MAX_INTEGRAL_POWER);
            double stiction = Math.copySign(
                    STICTION_POWER * clamp(
                            Math.abs(positionError) / STICTION_FADE_TICKS, 0, 1),
                    positionError);
            double controllerPower =
                    POSITION_KP * positionError
                            + VELOCITY_KD * velocityError
                            + VELOCITY_KV * targetVelocityTicksPerSecond
                            + integralPower + stiction;

            // Reduce every inner-loop term near the setpoint, as System 1 does.
            double proximityScale = clamp(
                    Math.abs(positionError) / FULL_POWER_ERROR_TICKS,
                    MIN_PROXIMITY_POWER_SCALE, 1.0);
            appliedPower = MOTOR_DIRECTION_SIGN * controllerPower * proximityScale;
        }

        appliedPower = limitAtSoftStops(
                clamp(appliedPower, -MAX_TURRET_POWER, MAX_TURRET_POWER), currentTicks);
        turretMotor.setPower(appliedPower);
    }

    private void updateFilteredMotorVelocity() {
        double rawVelocity = turretMotor.getVelocity();
        if (!motorVelocityInitialized) {
            filteredMotorVelocityTicksPerSecond = rawVelocity;
            motorVelocityInitialized = true;
            return;
        }
        filteredMotorVelocityTicksPerSecond += MOTOR_VELOCITY_FILTER_ALPHA
                * (rawVelocity - filteredMotorVelocityTicksPerSecond);
    }

    private void applyManualControl(int currentTicks) {
        targetTicksInitialized = false;
        integralPower = 0;
        appliedPower = limitAtSoftStops(manualPower, currentTicks);
        turretMotor.setPower(appliedPower);
    }

    private double limitAtSoftStops(double power, int currentTicks) {
        double tickDirection = MOTOR_DIRECTION_SIGN * power;
        if (currentTicks >= MAX_TURRET_TICKS && tickDirection > 0) return 0;
        if (currentTicks <= MIN_TURRET_TICKS && tickDirection < 0) return 0;
        return power;
    }

    private List<LLResultTypes.FiducialResult> selectClusterTags(
            List<LLResultTypes.FiducialResult> tags) {
        List<LLResultTypes.FiducialResult> empty = new ArrayList<>();
        if (tags == null || tags.isEmpty()) {
            selectedClusterRollDeg = Double.NaN;
            return empty;
        }

        int firstClusterId = alliance == Alliance.RED
                ? RED_SCORING_CLUSTER_FIRST_ID : BLUE_AUDIENCE_CLUSTER_FIRST_ID;
        int secondClusterId = alliance == Alliance.RED
                ? RED_AUDIENCE_CLUSTER_FIRST_ID : BLUE_SCORING_CLUSTER_FIRST_ID;
        List<LLResultTypes.FiducialResult> firstCluster = new ArrayList<>();
        List<LLResultTypes.FiducialResult> secondCluster = new ArrayList<>();

        for (LLResultTypes.FiducialResult tag : tags) {
            int clusterId = clusterFirstIdForTag(tag.getFiducialId());
            if (clusterId == firstClusterId) firstCluster.add(tag);
            if (clusterId == secondClusterId) secondCluster.add(tag);
        }

        ClusterCandidate first = createClusterCandidate(firstClusterId, firstCluster);
        ClusterCandidate second = createClusterCandidate(secondClusterId, secondCluster);
        ClusterCandidate selected = chooseScorableCluster(first, second);
        if (selected == null) {
            selectedClusterRollDeg = Double.NaN;
            return empty;
        }

        if (selectedClusterFirstId != selected.firstId) {
            selectedClusterFirstId = selected.firstId;
            targetTicksInitialized = false;
            resetMeasurementHistory();
        }
        selectedClusterRollDeg = selected.rollDeg;
        selectedTagId = Integer.MAX_VALUE;
        for (LLResultTypes.FiducialResult tag : selected.tags) {
            selectedTagId = Math.min(selectedTagId, tag.getFiducialId());
        }
        return selected.tags;
    }

    private ClusterCandidate createClusterCandidate(
            int firstId, List<LLResultTypes.FiducialResult> tags) {
        if (tags.isEmpty()) return null;

        double sinRoll = 0;
        double cosRoll = 0;
        double totalArea = 0;
        int rollSamples = 0;
        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag.getTargetPoseCameraSpace() == null
                    || tag.getTargetPoseCameraSpace().getOrientation() == null) continue;
            double rollDeg = tag.getTargetPoseCameraSpace().getOrientation()
                    .getRoll(AngleUnit.DEGREES);
            if (!Double.isFinite(rollDeg)) continue;
            double rollRad = Math.toRadians(rollDeg);
            sinRoll += Math.sin(rollRad);
            cosRoll += Math.cos(rollRad);
            totalArea += Math.max(0, tag.getTargetArea());
            rollSamples++;
        }
        if (rollSamples == 0) return null;

        double averageRollDeg = Math.toDegrees(Math.atan2(sinRoll, cosRoll));
        return new ClusterCandidate(firstId, tags, averageRollDeg, totalArea);
    }

    private ClusterCandidate chooseScorableCluster(
            ClusterCandidate first, ClusterCandidate second) {
        boolean firstScorable = first != null && Math.abs(first.rollDeg) < 90.0;
        boolean secondScorable = second != null && Math.abs(second.rollDeg) < 90.0;
        if (firstScorable && !secondScorable) return first;
        if (secondScorable && !firstScorable) return second;
        if (!firstScorable) return null;

        // Both should not normally be upright. Prefer the stronger observation.
        if (first.tags.size() != second.tags.size()) {
            return first.tags.size() > second.tags.size() ? first : second;
        }
        return first.totalArea >= second.totalArea ? first : second;
    }

    private static class ClusterCandidate {
        final int firstId;
        final List<LLResultTypes.FiducialResult> tags;
        final double rollDeg;
        final double totalArea;

        ClusterCandidate(int firstId, List<LLResultTypes.FiducialResult> tags,
                double rollDeg, double totalArea) {
            this.firstId = firstId;
            this.tags = tags;
            this.rollDeg = rollDeg;
            this.totalArea = totalArea;
        }
    }

    /** Horizontal aim to the official cluster origin, not to an individual tag. */
    private ClusterAim calculateClusterAim(
            List<LLResultTypes.FiducialResult> tags) {
        int count = tags.size();
        double sumTanTx = 0;
        double sumX = 0;
        double sumTy = 0;

        for (LLResultTypes.FiducialResult tag : tags) {
            double correctedTxDeg = tag.getTargetXDegrees()
                    - aimOffsetForTag(tag.getFiducialId());
            sumTanTx += Math.tan(Math.toRadians(correctedTxDeg));
            sumX += clusterXForTag(tag.getFiducialId());
            sumTy += tag.getTargetYDegrees();
        }

        double meanTanTx = sumTanTx / count;
        double meanX = sumX / count;
        double averageTyDeg = sumTy / count;
        double tagPlaneDepthIn = Double.NaN;
        double clusterLateralIn = Double.NaN;

        // tan(tx) = (cluster lateral + tag X) / tag-plane depth.
        // Two or more tags therefore provide a direct depth and center solution.
        if (count >= 2) {
            double covariance = 0;
            double xVariance = 0;
            for (LLResultTypes.FiducialResult tag : tags) {
                double x = clusterXForTag(tag.getFiducialId());
                double correctedTxDeg = tag.getTargetXDegrees()
                        - aimOffsetForTag(tag.getFiducialId());
                double tanTx = Math.tan(Math.toRadians(correctedTxDeg));
                covariance += (x - meanX) * (tanTx - meanTanTx);
                xVariance += (x - meanX) * (x - meanX);
            }
            double slope = xVariance > 1e-6 ? covariance / xVariance : 0;
            if (slope > 1.0 / MAX_DISTANCE_IN) {
                tagPlaneDepthIn = 1.0 / slope;
                if (tagPlaneDepthIn > CLUSTER_AIM_POINT_FORWARD_OF_TAGS_IN
                        + CLUSTER_AIM_POINT_ABOVE_TAGS_IN + 1.0
                        && tagPlaneDepthIn < MAX_DISTANCE_IN) {
                    clusterLateralIn = (meanTanTx - slope * meanX)
                            * tagPlaneDepthIn;
                    lastClusterTagPlaneDepthIn = tagPlaneDepthIn;
                } else {
                    tagPlaneDepthIn = Double.NaN;
                }
            }
        }

        // With one visible tag, use its 3D range. If unavailable, preserve the
        // most recent depth solved from multiple tags so partial occlusion does
        // not move the aim point to that tag.
        if (!Double.isFinite(tagPlaneDepthIn)) {
            double depthSum = 0;
            int depthCount = 0;
            for (LLResultTypes.FiducialResult tag : tags) {
                double depth = estimateOpticalDepthIn(tag);
                if (Double.isFinite(depth)) {
                    depthSum += depth;
                    depthCount++;
                }
            }
            tagPlaneDepthIn = depthCount > 0
                    ? depthSum / depthCount : lastClusterTagPlaneDepthIn;

            if (Double.isFinite(tagPlaneDepthIn)
                    && tagPlaneDepthIn > CLUSTER_AIM_POINT_FORWARD_OF_TAGS_IN
                            + CLUSTER_AIM_POINT_ABOVE_TAGS_IN + 1.0) {
                double lateralSum = 0;
                for (LLResultTypes.FiducialResult tag : tags) {
                    double correctedTxDeg = tag.getTargetXDegrees()
                            - aimOffsetForTag(tag.getFiducialId());
                    lateralSum += tagPlaneDepthIn
                            * Math.tan(Math.toRadians(correctedTxDeg))
                            - clusterXForTag(tag.getFiducialId());
                }
                clusterLateralIn = lateralSum / count;
                lastClusterTagPlaneDepthIn = tagPlaneDepthIn;
            }
        }

        clusterGeometryValid = Double.isFinite(clusterLateralIn)
                && Double.isFinite(tagPlaneDepthIn);
        double centerTxDeg;
        double horizontalDistanceIn = Double.NaN;
        if (clusterGeometryValid) {
            // SDK cluster coordinates put each tag forward and below the CELL
            // opening origin. Project that 3-D offset into the pitched camera.
            double cameraPitchRad = Math.toRadians(CAMERA_DOWN_ANGLE_DEG);
            double tagToOriginDepthIn = CLUSTER_AIM_POINT_FORWARD_OF_TAGS_IN
                    * Math.cos(cameraPitchRad)
                    + CLUSTER_AIM_POINT_ABOVE_TAGS_IN * Math.sin(cameraPitchRad);
            double tagToOriginVerticalIn = CLUSTER_AIM_POINT_FORWARD_OF_TAGS_IN
                    * Math.sin(cameraPitchRad)
                    - CLUSTER_AIM_POINT_ABOVE_TAGS_IN * Math.cos(cameraPitchRad);
            double aimPointDepthIn = tagPlaneDepthIn - tagToOriginDepthIn;
            centerTxDeg = Math.toDegrees(
                    Math.atan2(clusterLateralIn, aimPointDepthIn));

            double tagVerticalOpticalIn = tagPlaneDepthIn
                    * Math.tan(Math.toRadians(averageTyDeg));
            double aimPointVerticalOpticalIn = tagVerticalOpticalIn
                    - tagToOriginVerticalIn;
            double aimPointForwardIn = aimPointDepthIn * Math.cos(cameraPitchRad)
                    + aimPointVerticalOpticalIn * Math.sin(cameraPitchRad);
            horizontalDistanceIn = Math.hypot(
                    aimPointForwardIn, clusterLateralIn);
        } else {
            // Safe startup fallback if Limelight 3D is disabled and only one tag
            // has ever been visible. Multiple visible tags use their angular mean.
            centerTxDeg = Math.toDegrees(Math.atan(meanTanTx));
        }
        return new ClusterAim(
                centerTxDeg, averageTyDeg, horizontalDistanceIn);
    }

    private double estimateOpticalDepthIn(LLResultTypes.FiducialResult tag) {
        if (tag.getTargetPoseCameraSpace() == null
                || tag.getTargetPoseCameraSpace().getPosition() == null) {
            return Double.NaN;
        }
        org.firstinspires.ftc.robotcore.external.navigation.Position position =
                tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
        double slantRangeIn = Math.sqrt(position.x * position.x
                + position.y * position.y + position.z * position.z);
        if (!Double.isFinite(slantRangeIn) || slantRangeIn < 1.0
                || slantRangeIn >= MAX_DISTANCE_IN) {
            return Double.NaN;
        }
        double tanTx = Math.tan(Math.toRadians(tag.getTargetXDegrees()));
        double tanTy = Math.tan(Math.toRadians(tag.getTargetYDegrees()));
        return slantRangeIn / Math.sqrt(1.0 + tanTx * tanTx + tanTy * tanTy);
    }

    private double clusterXForTag(int id) {
        int index = indexOfTag(id);
        return index >= 0 && index < CLUSTER_TAG_X_OFFSETS_IN.length
                ? CLUSTER_TAG_X_OFFSETS_IN[index] : 0;
    }

    private static class ClusterAim {
        final double txDeg;
        final double averageTyDeg;
        final double horizontalDistanceIn;

        ClusterAim(
                double txDeg, double averageTyDeg, double horizontalDistanceIn) {
            this.txDeg = txDeg;
            this.averageTyDeg = averageTyDeg;
            this.horizontalDistanceIn = horizontalDistanceIn;
        }
    }

    private boolean isAllowedTag(int id) {
        if (alliance == Alliance.RED) return id >= 30 && id <= 37;
        return id >= 38 && id <= 45;
    }

    private int clusterFirstIdForTag(int id) {
        if (id < 30 || id > 45) return -1;
        return 30 + ((id - 30) / 4) * 4;
    }

    private int indexOfTag(int id) {
        int index = id - selectedClusterFirstId;
        return index >= 0 && index < 4 ? index : -1;
    }

    private double aimOffsetForTag(int id) {
        int index = indexOfTag(id);
        return index >= 0 && index < TAG_AIM_OFFSETS_DEG.length
                ? TAG_AIM_OFFSETS_DEG[index] : 0;
    }

    private double tagHeightForId(int id) {
        int index = indexOfTag(id);
        return index >= 0 && index < TAG_HEIGHTS_IN.length
                ? TAG_HEIGHTS_IN[index] : DEFAULT_TAG_HEIGHT_IN;
    }

    private void updateVisibleTagText(List<LLResultTypes.FiducialResult> tags) {
        if (tags == null || tags.isEmpty()) {
            visibleTagIds = "none";
            return;
        }
        StringBuilder ids = new StringBuilder();
        for (LLResultTypes.FiducialResult tag : tags) {
            if (!isAllowedTag(tag.getFiducialId())) continue;
            if (ids.length() > 0) ids.append(", ");
            ids.append(tag.getFiducialId());
        }
        visibleTagIds = ids.length() == 0 ? "none allowed" : ids.toString();
    }

    private void saveEncoderSample(double nowMs, int ticks) {
        encoderHistory.addLast(new double[]{nowMs, ticks});
        while (!encoderHistory.isEmpty()
                && nowMs - encoderHistory.peekFirst()[0] > ENCODER_HISTORY_MS) {
            encoderHistory.removeFirst();
        }
    }

    private double encoderTicksAt(double timeMs) {
        double[] previous = null;
        for (double[] sample : encoderHistory) {
            if (sample[0] >= timeMs) {
                if (previous == null) return sample[1];
                double spanMs = Math.max(sample[0] - previous[0], 0.001);
                double fraction = (timeMs - previous[0]) / spanMs;
                return previous[1] + fraction * (sample[1] - previous[1]);
            }
            previous = sample;
        }
        return previous == null ? robot.getTurretPositionTicks() : previous[1];
    }

    private double addAndMedian(double[] history, int count, double value) {
        for (int i = history.length - 1; i > 0; i--) history[i] = history[i - 1];
        history[0] = value;
        double[] sorted = Arrays.copyOf(history, Math.min(count + 1, history.length));
        Arrays.sort(sorted);
        return sorted[sorted.length / 2];
    }

    private double removeRateDeadband(double rate) {
        return Math.abs(rate) > RATE_DEADBAND_DEG_PER_SEC
                ? rate - Math.copySign(RATE_DEADBAND_DEG_PER_SEC, rate) : 0;
    }

    private void resetMeasurementHistory() {
        lastFrameTimestamp = -1;
        txHistoryCount = 0;
        distanceHistoryCount = 0;
        distanceInitialized = false;
        bearingInitialized = false;
        rangeRateInitialized = false;
        lastBearingTimeMs = -1;
        lastDistanceTimeMs = -1;
        targetRateDegPerSec = 0;
        rangeRateInPerSec = 0;
        lastRobotBearingDeg = Double.NaN;
        odometryTargetInitialized = false;
        odometryTrackingActive = false;
        odometryTrackedBearingDeg = 0;
        odometryTrackedDistanceIn = 0;
        lastClusterTagPlaneDepthIn = Double.NaN;
        clusterGeometryValid = false;
        panLeadDeg = 0;
        aimReady = false;
        stableAimFrames = 0;
    }

    public boolean hasTarget() { return targetVisible; }
    public boolean hasDistance() { return distanceInitialized; }
    public boolean isAimReady() { return aimReady; }
    public int getSelectedTagId() { return selectedTagId; }
    public Alliance getAlliance() { return alliance; }
    public int getSelectedClusterFirstId() { return selectedClusterFirstId; }
    public double getSelectedClusterRollDeg() { return selectedClusterRollDeg; }
    public String getSelectedClusterName() {
        switch (selectedClusterFirstId) {
            case RED_SCORING_CLUSTER_FIRST_ID: return "RED SCORING";
            case RED_AUDIENCE_CLUSTER_FIRST_ID: return "RED AUDIENCE";
            case BLUE_AUDIENCE_CLUSTER_FIRST_ID: return "BLUE AUDIENCE";
            case BLUE_SCORING_CLUSTER_FIRST_ID: return "BLUE SCORING";
            default: return "NONE";
        }
    }
    public String getVisibleTagIds() { return visibleTagIds; }
    public int getVisibleTagCount() { return visibleTagCount; }
    public boolean isClusterGeometryValid() { return clusterGeometryValid; }
    public double getFilteredTxDeg() { return filteredTxDeg; }
    public double getDistanceIn() { return filteredDistanceIn; }
    public double getImpactDistanceIn() { return impactDistanceIn; }
    public double getPanLeadDeg() { return panLeadDeg; }
    /** Rate actually used by prediction after optional vision/odometry blending. */
    public double getTargetRateDegPerSec() { return getPredictionTargetRateDegPerSec(); }
    public double getRangeRateInPerSec() { return getPredictionRangeRateInPerSec(); }
    public double getVisionTargetRateDegPerSec() { return targetRateDegPerSec; }
    public double getVisionRangeRateInPerSec() { return rangeRateInPerSec; }
    public double getOdometryTargetRateDegPerSec() { return odometryTargetRateDegPerSec; }
    public double getOdometryRangeRateInPerSec() { return odometryRangeRateInPerSec; }
    public double getRobotForwardVelocityInPerSec() { return robotForwardVelocityInPerSec; }
    public double getRobotLeftVelocityInPerSec() { return robotLeftVelocityInPerSec; }
    public double getRobotHeadingRateDegPerSec() {
        return Math.toDegrees(robotHeadingRateRadPerSec);
    }
    public boolean isOdometryRateValid() { return odometryRateValid; }
    public boolean isOdometryPoseValid() { return odometryPoseValid; }
    public boolean isOdometryTrackingActive() { return odometryTrackingActive; }
    public double getOdometryTrackedBearingDeg() { return odometryTrackedBearingDeg; }
    public double getOdometryTrackedDistanceIn() { return odometryTrackedDistanceIn; }
    public double getTargetAgeMs() {
        return lastTargetSeenMs < 0
                ? Double.POSITIVE_INFINITY : clock.milliseconds() - lastTargetSeenMs;
    }
    public double getPredictionTimeS() { return predictionTimeS; }
    public double getLastVisionErrorDeg() { return lastVisionErrorDeg; }
    public double getLastLatencyMs() { return lastLatencyMs; }
    public int getCurrentTicks() { return robot.getTurretPositionTicks(); }
    public double getTargetTicks() { return targetTicks; }
    public double getPositionErrorTicks() {
        return targetTicks - robot.getTurretPositionTicks();
    }
    public double getFilteredMotorVelocityTicksPerSecond() {
        return filteredMotorVelocityTicksPerSecond;
    }
    public double getIntegralPower() { return integralPower; }
    public double getAppliedPower() { return appliedPower; }

    public void stop() {
        turretMotor.setPower(0);
        robot.limelight.stop();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
