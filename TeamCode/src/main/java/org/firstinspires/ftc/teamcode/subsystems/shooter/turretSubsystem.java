package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Deque;
import java.util.List;

/** Owns AprilTag tracking, motion prediction, and turret motor control. */
public class turretSubsystem {

    // ========================= VALUES TO CALIBRATE =========================

    /** Limelight AprilTag pipeline number. */
    public static final int APRILTAG_PIPELINE = 0;

    /**
     * Enter this season's four tag IDs. Empty accepts every AprilTag for initial testing.
     * Example: {1, 2, 3, 4}.
     */
    public static final int[] TARGET_TAG_IDS = {38, 39, 40, 41};

    /** Per-tag horizontal corrections, in the same order. Positive moves the aim point left. */
    public static final double[] TAG_AIM_OFFSETS_DEG = {};

    /** Optional per-tag heights, in the same order. Empty uses DEFAULT_TAG_HEIGHT_IN. */
    public static final double[] TAG_HEIGHTS_IN = {};

    /** Measure encoder ticks for a known turret turn, then divide ticks by degrees. */
    public static final double TICKS_PER_DEGREE = 2.04;

    /** Flip this sign if the turret corrects away from the target. */
    public static final double AIM_DIRECTION_SIGN = 1.0;

    /** Flip this sign if motor power and encoder tick direction disagree. */
    public static final double MOTOR_DIRECTION_SIGN = 1.0;

    /** Encoder safety limits; encoder zero must be the physical center. */
    public static final int MIN_TURRET_TICKS = -300;
    public static final int MAX_TURRET_TICKS = 300;
    public static final double MAX_TURRET_POWER = 0.5;

    /** Camera geometry for calculating horizontal floor distance. */
    public static final double CAMERA_HEIGHT_IN = 48.0;
    public static final double DEFAULT_TAG_HEIGHT_IN = 14.5;
    public static final double CAMERA_DOWN_ANGLE_DEG = 24.24;

    /** Encoder controller: tune P first, D second, I last. */
    public static final double POSITION_KP = 0.045;
    public static final double VELOCITY_KD = 0.0;
    public static final double VELOCITY_KV = 0.0;
    public static final double POSITION_KI = 0.0;
    public static final double MAX_INTEGRAL_POWER = 0.10;
    public static final double STICTION_POWER = 0.0;

    /** Smaller is smoother; larger reacts faster to each new camera measurement. */
    public static final double VISION_CORRECTION_GAIN = 0.18;
    public static final double AIM_TOLERANCE_DEG = 0.30;
    public static final int AIM_STABLE_FRAMES = 3;

    /** Motion prediction: tune flight time after basic stationary aiming works. */
    public static final boolean PREDICTION_ENABLED = false;
    public static final double FIXED_FLIGHT_TIME_S = 0.12;
    public static final double TOF_PER_INCH_S = 0.0055;
    public static final double FIXED_SHOT_DELAY_S = 0.06;
    public static final double MAX_PREDICTION_TIME_S = 1.20;
    public static final double MAX_ANGLE_LEAD_DEG = 12.0;
    public static final double MAX_DISTANCE_LEAD_IN = 36.0;

    /**
     * Enable only after the Pinpoint position, velocity, and signs have been verified in telemetry.
     * Odometry is blended with vision; it is not added, because vision already sees robot motion.
     */
    public static final boolean USE_ODOMETRY_PREDICTION = false;
    public static final double ODOMETRY_RATE_BLEND = 0.70;

    /** Flip individual signs if Pinpoint telemetry disagrees with actual robot motion. */
    public static final double ODOMETRY_FORWARD_SIGN = 1.0;
    public static final double ODOMETRY_LEFT_SIGN = 1.0;
    public static final double ODOMETRY_HEADING_SIGN = 1.0;

    /** Converts Limelight's clockwise-positive bearing into counterclockwise-positive math. */
    public static final double ODOMETRY_BEARING_SIGN = -1.0;

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
    private static final double SETPOINT_DEADBAND_TICKS = 0.6;
    private static final double STICTION_FADE_TICKS = 5.0;

    private final Hardware robot;
    private final DcMotorEx turretMotor;
    private final ElapsedTime clock = new ElapsedTime();
    private final Deque<double[]> encoderHistory = new ArrayDeque<>();
    private final double[] txHistory = new double[3];
    private final double[] distanceHistory = new double[5];

    private boolean autoAimEnabled;
    private double manualPower;
    private int selectedTagId = -1;
    private String visibleTagIds = "none";
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
    private double odometryTargetRateDegPerSec;
    private double odometryRangeRateInPerSec;
    private double lastRobotBearingDeg = Double.NaN;
    private boolean odometryRateValid;

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

    public turretSubsystem(Hardware robot) {
        this.robot = robot;
        turretMotor = robot.turret;
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
    }

    public void setManualPower(double power) {
        manualPower = clamp(power, -MAX_TURRET_POWER, MAX_TURRET_POWER);
    }

    /** Call once per OpMode loop. */
    public void update(double dtSeconds) {
        double dt = clamp(dtSeconds, 0.001, 0.100);
        double nowMs = clock.milliseconds();
        int currentTicks = turretMotor.getCurrentPosition();

        updateOdometry();
        saveEncoderSample(nowMs, currentTicks);
        updateVision(nowMs, currentTicks);
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
        LLResultTypes.FiducialResult target = selectTarget(tags);
        targetVisible = target != null;

        if (!targetVisible) {
            aimReady = false;
            stableAimFrames = 0;
            targetRateDegPerSec *= 0.90;
            panLeadDeg = 0;
            if (lastTargetSeenMs >= 0 && nowMs - lastTargetSeenMs > HOLD_AFTER_TARGET_LOSS_MS) {
                selectedTagId = -1;
                resetMeasurementHistory();
            }
            return;
        }

        double frameTimestamp = result.getTimestamp();
        if (frameTimestamp == lastFrameTimestamp) return;
        lastFrameTimestamp = frameTimestamp;
        lastTargetSeenMs = nowMs;

        filteredTxDeg = addAndMedian(txHistory, txHistoryCount, target.getTargetXDegrees());
        txHistoryCount = Math.min(txHistoryCount + 1, txHistory.length);
        updateDistance(target, selectedTagId);

        lastLatencyMs = result.getStaleness() + result.getCaptureLatency()
                + result.getTargetingLatency() + EXTRA_LATENCY_MS;
        double captureTimeMs = nowMs - lastLatencyMs;
        double ticksAtCapture = encoderTicksAt(captureTimeMs);
        double turretMovementDeg = (currentTicks - ticksAtCapture) / TICKS_PER_DEGREE;
        double horizontalErrorDeg = filteredTxDeg - aimOffsetForTag(selectedTagId);

        // Subtract turret rotation so rate tracks target/chassis movement, not turret movement.
        double bearingAtCaptureDeg = horizontalErrorDeg - ticksAtCapture / TICKS_PER_DEGREE;
        lastRobotBearingDeg = bearingAtCaptureDeg;
        updateTargetRates(captureTimeMs, bearingAtCaptureDeg);
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
                targetTicks += VISION_CORRECTION_GAIN * (measuredTargetTicks - targetTicks);
            }
        }
    }

    private void updateDistance(LLResultTypes.FiducialResult target, int tagId) {
        double depressionAngleDeg = CAMERA_DOWN_ANGLE_DEG - target.getTargetYDegrees();
        if (depressionAngleDeg <= MIN_DISTANCE_ANGLE_DEG) return;

        double rawDistanceIn = (CAMERA_HEIGHT_IN - tagHeightForId(tagId))
                / Math.tan(Math.toRadians(depressionAngleDeg));
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

    /** Reads Pinpoint field velocity and rotates it into robot-forward/robot-left velocity. */
    private void updateOdometry() {
        robot.odo.update();

        double fieldVelocityX = robot.odo.getVelX(DistanceUnit.INCH);
        double fieldVelocityY = robot.odo.getVelY(DistanceUnit.INCH);
        double headingRad = robot.odo.getHeading(AngleUnit.RADIANS);
        double headingRateDegPerSec = robot.odo.getHeadingVelocity(
                UnnormalizedAngleUnit.DEGREES);

        odometryRateValid = Double.isFinite(fieldVelocityX)
                && Double.isFinite(fieldVelocityY)
                && Double.isFinite(headingRad)
                && Double.isFinite(headingRateDegPerSec);
        if (!odometryRateValid) return;

        double cosHeading = Math.cos(headingRad);
        double sinHeading = Math.sin(headingRad);
        robotForwardVelocityInPerSec = ODOMETRY_FORWARD_SIGN
                * (fieldVelocityX * cosHeading + fieldVelocityY * sinHeading);
        robotLeftVelocityInPerSec = ODOMETRY_LEFT_SIGN
                * (-fieldVelocityX * sinHeading + fieldVelocityY * cosHeading);
        robotHeadingRateRadPerSec = ODOMETRY_HEADING_SIGN
                * Math.toRadians(headingRateDegPerSec);
    }

    /** Projects robot velocity along and across the line of sight to the selected tag. */
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

        // Positive range rate means distance is increasing.
        odometryRangeRateInPerSec = clamp(
                -velocityTowardTarget,
                -MAX_RANGE_RATE_IN_PER_SEC,
                MAX_RANGE_RATE_IN_PER_SEC);

        // Target bearing changes opposite chassis rotation and lateral translation.
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

        if (lastTargetSeenMs < 0 || nowMs - lastTargetSeenMs > HOLD_AFTER_TARGET_LOSS_MS) {
            targetTicks = currentTicks;
            targetVelocityTicksPerSecond = 0;
            integralPower = 0;
        } else {
            targetVelocityTicksPerSecond = AIM_DIRECTION_SIGN
                    * removeRateDeadband(getPredictionTargetRateDegPerSec())
                    * TICKS_PER_DEGREE;
        }

        targetTicks = clamp(targetTicks, MIN_TURRET_TICKS, MAX_TURRET_TICKS);
        double positionError = targetTicks - currentTicks;
        double velocityError = targetVelocityTicksPerSecond - turretMotor.getVelocity();

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
            appliedPower = MOTOR_DIRECTION_SIGN * (
                    POSITION_KP * positionError
                            + VELOCITY_KD * velocityError
                            + VELOCITY_KV * targetVelocityTicksPerSecond
                            + integralPower + stiction);
        }

        appliedPower = limitAtSoftStops(
                clamp(appliedPower, -MAX_TURRET_POWER, MAX_TURRET_POWER), currentTicks);
        turretMotor.setPower(appliedPower);
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

    /** Keeps the previous ID while visible; otherwise acquires the tag nearest image center. */
    private LLResultTypes.FiducialResult selectTarget(
            List<LLResultTypes.FiducialResult> tags) {
        if (tags == null || tags.isEmpty()) return null;

        LLResultTypes.FiducialResult best = null;
        for (LLResultTypes.FiducialResult tag : tags) {
            if (!isAllowedTag(tag.getFiducialId())) continue;
            if (tag.getFiducialId() == selectedTagId) return tag;
            if (best == null || Math.abs(tag.getTargetXDegrees())
                    < Math.abs(best.getTargetXDegrees())) {
                best = tag;
            }
        }

        if (best != null && best.getFiducialId() != selectedTagId) {
            selectedTagId = best.getFiducialId();
            resetMeasurementHistory();
            targetTicksInitialized = false;
        }
        return best;
    }

    private boolean isAllowedTag(int id) {
        if (TARGET_TAG_IDS.length == 0) return true;
        for (int allowedId : TARGET_TAG_IDS) {
            if (allowedId == id) return true;
        }
        return false;
    }

    private int indexOfTag(int id) {
        for (int i = 0; i < TARGET_TAG_IDS.length; i++) {
            if (TARGET_TAG_IDS[i] == id) return i;
        }
        return -1;
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
        return previous == null ? turretMotor.getCurrentPosition() : previous[1];
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
        panLeadDeg = 0;
        aimReady = false;
        stableAimFrames = 0;
    }

    public boolean hasTarget() { return targetVisible; }
    public boolean hasDistance() { return distanceInitialized; }
    public boolean isAimReady() { return aimReady; }
    public int getSelectedTagId() { return selectedTagId; }
    public String getVisibleTagIds() { return visibleTagIds; }
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
    public boolean isOdometryRateValid() { return odometryRateValid; }
    public double getPredictionTimeS() { return predictionTimeS; }
    public double getLastVisionErrorDeg() { return lastVisionErrorDeg; }
    public double getLastLatencyMs() { return lastLatencyMs; }
    public int getCurrentTicks() { return turretMotor.getCurrentPosition(); }
    public double getTargetTicks() { return targetTicks; }
    public double getAppliedPower() { return appliedPower; }

    public void stop() {
        turretMotor.setPower(0);
        robot.limelight.stop();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
