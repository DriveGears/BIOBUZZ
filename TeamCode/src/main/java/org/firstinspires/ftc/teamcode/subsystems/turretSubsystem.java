package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.OdometryConfig;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Deque;
import java.util.List;

/** Owns AprilTag/odometry hive tracking and turret motor control. */
@Config
public class turretSubsystem {

    // ========================= VALUES TO CALIBRATE =========================

    /** Displayed by the test TeleOp so the deployed tuning profile is verifiable. */
    public static final String TUNE_VERSION = "clean-v8-continuous-side-bank";

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

    /**
     * Symmetric angular calibration. Values above 1.0 aim progressively farther
     * from robot-forward on both sides; camera-to-shooter parallax remains separate.
     * Keep at 1.0 until the turret PID and mechanics are verified.
     */
    public static double AIM_ANGLE_SCALE = 1.05;

    /** Optional per-tag heights, in the same order. Empty uses DEFAULT_TAG_HEIGHT_IN. */
    public static final double[] TAG_HEIGHTS_IN = {};

    /** Encoder ticks per degree (System 1 calibrated value). */
    public static final double TICKS_PER_DEGREE = 1.81;

    /** Flip this sign if the turret corrects away from the target. */
    public static final double AIM_DIRECTION_SIGN = 1.0;

    /** Flip this sign if motor power and encoder tick direction disagree. */
    public static final double MOTOR_DIRECTION_SIGN = 1.0;

    /** Encoder safety limits. */
    public static final int MIN_TURRET_TICKS = -100;
    public static final int MAX_TURRET_TICKS = 400;
    public static final double MAX_TURRET_POWER = 0.85;
    /** Prevent a noisy blind-sector bearing from alternating between both soft stops. */
    public static final double BLIND_SECTOR_SWITCH_HYSTERESIS_DEG = 6.0;
    private static final double WRAP_ROUTE_MIN_VELOCITY_TPS = 5.0;
    /** One physical turret revolution, used to choose a reachable equivalent angle. */
    public static final double TURRET_TICKS_PER_REVOLUTION = 360.0 * TICKS_PER_DEGREE;

    /**
     * Camera geometry for calculating horizontal floor distance. These are deliberately
     * non-final so they can be calibrated from FTC Dashboard while normal TeleOp runs.
     * Pitch is positive UP from horizontal; it is the Limelight mount angle, not the
     * 30-degree HIVE tag-plane angle.
     */
    public static double CAMERA_HEIGHT_IN = 14.0;
    public static double DEFAULT_TAG_HEIGHT_IN = 50.9;
    public static double CAMERA_UP_ANGLE_DEG = 24.24;

    /**
     * Physical horizontal separation between the parallel camera and shooter axes.
     * Positive means the Limelight is to the shooter's left when looking forward.
     */
    public static final double CAMERA_LEFT_OF_SHOOTER_IN = 3.5;

    /**
     * Location-based bank aim. The known-good left shot is unchanged. Starting
     * near the hive center, left correction grows continuously until the shooter
     * is SIDE_AIM_FULL_RIGHT_IN to the right of the hive.
     */
    public static double RIGHT_POSITION_AIM_LEFT_IN = 14.0;
    public static double SIDE_AIM_START_RIGHT_IN = 0.0;
    public static double SIDE_AIM_FULL_RIGHT_IN = 8.0;
    public static double SIDE_POSITION_FILTER_ALPHA = 0.25;
    public static double BANK_AIM_FILTER_ALPHA = 0.25;

    /**
     * Official BIOBUZZ tag X positions relative to the CELL opening. Camera-to-shooter
     * displacement is applied separately so its correction changes correctly with range.
     */
    public static final double[] CLUSTER_TAG_X_OFFSETS_IN = {
            -6.50, -2.75, 2.75, 6.50
    };
    public static final double CLUSTER_AIM_POINT_FORWARD_OF_TAGS_IN = 7.1874;
    /** Official SDK Z offset: tag centers are this far below the opening origin. */
    public static final double CLUSTER_AIM_POINT_ABOVE_TAGS_IN = 5.622;

    /**
     * Clean PID starting point for the rebuilt mechanism. These are deliberately
     * Dashboard-tunable and contain no friction-specific compensation yet.
     */
    public static double POSITION_KP = 0.040;
    public static double POSITION_KI = 0.0;
    public static double VELOCITY_KD = 0.0030;
    public static double VELOCITY_KV = 0.0;
    /** Small breakaway assist for the rebuilt belt drive. */
    public static double STATIC_FRICTION_POWER = 0.05;
    public static double MAX_INTEGRAL_POWER = 0.08;
    public static double INTEGRAL_ZONE_TICKS = 12.0;
    public static double POSITION_TOLERANCE_TICKS = 1.5;
    public static final double MOTOR_VELOCITY_FILTER_ALPHA = 0.50;
    /**
     * If the turret cannot move near the target, stop driving it instead of
     * heating the motor indefinitely. A moved setpoint automatically retries.
     */
    public static double NEAR_TARGET_STALL_WINDOW_TICKS = 8.0;
    public static double NEAR_TARGET_STALL_MAX_VELOCITY_TPS = 3.0;
    public static double NEAR_TARGET_STALL_TIMEOUT_MS = 500.0;
    public static double NEAR_TARGET_STALL_RETRY_DELTA_TICKS = 3.0;

    /**
     * Adaptive vision gain: big error -> trust the new frame almost fully
     * (fast snap after the robot stops), small error -> heavier smoothing.
     */
    public static final double VISION_GAIN_MIN = 0.30;
    public static final double VISION_GAIN_MAX = 0.85;
    public static final double VISION_GAIN_FULL_ERROR_DEG = 4.0;
    /**
     * Ignore vision corrections smaller than this while the turret is nearly
     * still, so +-1 tick of measurement/latency noise cannot keep re-kicking
     * the motor (v3 log: ~7% power sign flips and ~350 ms of hunting after arrival).
     */
    public static final double VISION_DEADBAND_TICKS = 2.0;
    public static final double VISION_DEADBAND_MAX_VELOCITY_TPS = 40.0;

    /** Ready = vision error small for N frames AND turret physically settled. */
    public static final double AIM_TOLERANCE_DEG = 0.90;
    public static final int AIM_STABLE_FRAMES = 2;
    public static final double READY_POSITION_TOL_TICKS = 2.0;
    public static final double READY_MAX_VELOCITY_TPS = 60.0;

    /** Median window for tx (frames). 5 added ~2 frames of lag. */
    private static final int TX_MEDIAN_WINDOW = 3;

    /**
     * Uses the encoder position at the moment the camera frame was captured,
     * so a moving turret does not over-correct from a stale image. This is what
     * allows the higher vision gains above without oscillation.
     */
    public static boolean LATENCY_COMPENSATION_ENABLED = true;

    /**
     * Store the observed hive as one field-space point. Vision corrects that
     * point; odometry converts it back to a live robot-relative bearing every
     * loop, including while the tag is hidden or the robot crosses the field.
     */
    public static boolean ODOMETRY_FUSION_ENABLED = true;
    /** How much of each new camera measurement is blended into the field target. */
    public static final double FUSION_MEASUREMENT_GAIN = 0.35;
    /**
     * While the robot is STILL the fused target should be averaged heavily and not chase +-1 deg of
     * camera noise (2026-11-13 log: setpoint sd 1.5 deg, turret power flipping 17x/s after a stop).
     * Gain starts at STILL_MIN and rises with the size of the disagreement so real changes still converge.
     */
    public static final double FUSION_STILL_GAIN_MIN = 0.08;
    public static final double FUSION_STILL_GAIN_PER_DEG = 0.12;
    /** When the robot is still, hold the setpoint unless the fused target moved more than this. */
    public static final double FUSION_STILL_DEADBAND_TICKS = 2.5;
    /** Large target jumps must agree for several frames before replacing hive memory. */
    public static final double FUSION_REJECT_DEG = 5.0;
    public static final double FUSION_REJECT_IN = 10.0;
    public static final int FUSION_RESEED_CONFIRM_FRAMES = 3;
    public static final double FUSION_CONFIRM_MATCH_IN = 8.0;
    public static final double FUSION_CONFIRM_MAX_GAP_MS = 200.0;
    /** Likewise, do not switch to the other hive cluster from one incorrect tag frame. */
    public static final int CLUSTER_SWITCH_CONFIRM_FRAMES = 3;
    private static final double FUSION_RATE_FILTER_ALPHA = 0.35;
    private static final double POSE_HISTORY_MS = 600.0;
    /**
     * Odometry sample guards (v5 calibration log: the X pod reported
     * FAULT_X_POD_NOT_DETECTED and caught up in 5-10 in jumps, plus BAD_READ blips).
     * A sample is rejected when the Pinpoint status is not READY, or the pose jumps
     * farther than physically possible between reads.
     */
    private static final double ODO_MAX_SPEED_IN_PER_S = 80.0;
    private static final double ODO_JUMP_SLACK_IN = 1.5;
    private static final int ODO_RESYNC_AFTER_JUMPS = 3;
    /** Pose is declared invalid when no good sample arrived for this long. */
    private static final double ODO_STALE_MS = 300.0;

    /** Robot counts as "stopped" below both of these (used for stop->ready timing). */
    public static final double STOPPED_SPEED_IN_PER_S = 2.0;
    public static final double STOPPED_HEADING_DEG_PER_S = 5.0;
    /** Must exceed these to count as moving again (hysteresis), and hold for the debounce time. */
    public static final double MOVING_SPEED_IN_PER_S = 5.0;
    public static final double MOVING_HEADING_DEG_PER_S = 12.0;
    public static final double STOP_DEBOUNCE_MS = 80.0;

    /**
     * Pinpoint I2C costs ~5 ms per read (measured), the biggest loop item.
     * Reading every Nth loop keeps odometry fresh enough (<~30 ms old) while
     * freeing ~3 ms/loop on average.
     */
    public static final int ODOMETRY_READ_EVERY_N_LOOPS = 2;

    /** Odometry sign settings. */
    public static double ODOMETRY_FORWARD_SIGN = 1.0;
    public static double ODOMETRY_LEFT_SIGN = 1.0;
    public static double ODOMETRY_HEADING_SIGN = 1.0;
    public static double ODOMETRY_BEARING_SIGN = -1.0;
    // ========================= NORMAL CONSTANTS =========================

    private static final long MAX_RESULT_AGE_MS = 120;
    private static final long HOLD_AFTER_TARGET_LOSS_MS = 350;
    private static final double MIN_DISTANCE_ANGLE_DEG = 2.0;
    /** Includes the roughly 204-inch diagonal of the 12-foot field, plus measurement margin. */
    private static final double MAX_DISTANCE_IN = 230.0;
    private static final double DISTANCE_FILTER_ALPHA = 0.25;
    /**
     * Ty range is responsive enough for RPM but too noisy to recompute horizontal
     * camera parallax directly every frame. Filter only the depth used for aiming.
     */
    private static final double AIM_DEPTH_FILTER_ALPHA_STILL = 0.12;
    private static final double AIM_DEPTH_FILTER_ALPHA_MOVING = 0.65;
    private static final double AIM_DEPTH_RESEED_DELTA_IN = 36.0;
    /** A stationary robot cannot physically change range this far between vision frames. */
    private static final double STATIONARY_DISTANCE_OUTLIER_IN = 24.0;
    /** Even at full drive speed, a larger frame-to-frame jump is a bad edge-tag reading. */
    private static final double MOVING_DISTANCE_OUTLIER_IN = 40.0;
    private static final double MAX_TARGET_RATE_DEG_PER_SEC = 60.0;
    private static final double EXTRA_LATENCY_MS = 10.0;
    private static final double ENCODER_HISTORY_MS = 400.0;
    /**
     * goBILDA RGB Indicator Light solid colors. With the configured 500-2500 us
     * servo range, its solid-color hue band is 1050-1950 us. Keeping every state
     * inside that band prevents the light from selecting an off/animation region.
     *
     * Blue   = manual aim
     * Red    = auto aim is not settled
     * Yellow = settled using saved odometry target; no live Limelight confidence
     * Green  = settled with a stable live Limelight target
     */
    public static final double LED_MANUAL_POSITION = 0.575;       // 1650 us: blue
    private static final double LED_ALIGNING_POSITION = 0.275;    // 1050 us: red
    private static final double LED_ODOMETRY_POSITION = 0.350;    // 1200 us: yellow
    private static final double LED_VISION_LOCK_POSITION = 0.425; // 1350 us: green
    private static final double LED_MIN_POSITION_CHANGE = 0.004;

    private final Hardware robot;
    private final DcMotorEx turretMotor;
    private final ElapsedTime clock = new ElapsedTime();
    private final Deque<double[]> encoderHistory = new ArrayDeque<>();
    private final double[] txHistory = new double[TX_MEDIAN_WINDOW];
    private final double[] distanceHistory = new double[5];

    private boolean autoAimEnabled;
    private double manualPower;
    private int selectedTagId = -1;
    private Alliance alliance = Alliance.BLUE;
    private int selectedClusterFirstId = -1;
    private double selectedClusterRollDeg = Double.NaN;
    private double cameraSideOfClusterIn = Double.NaN;
    private double shooterSideOfClusterIn = Double.NaN;
    private double bankAimOffsetIn;
    /** Current angular correction applied after center-of-hive odometry fusion. */
    private double bankAimCorrectionDeg;
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

    private double robotForwardVelocityInPerSec;
    private double robotLeftVelocityInPerSec;
    private double robotHeadingRateRadPerSec;
    private double robotFieldXIn;
    private double robotFieldYIn;
    private double robotFieldHeadingRad;
    private double lastRobotBearingDeg = Double.NaN;
    private boolean odometryRateValid;
    private boolean odometryPoseValid;

    /** True while a saved field-space hive is driving aim without live vision. */
    private boolean odometryTrackingActive;

    private boolean targetVisible;
    private boolean aimReady;
    private int stableAimFrames;
    private double lastVisionErrorDeg;
    private double lastLatencyMs;

    private double targetTicks;
    /** Reachable encoder representation of targetTicks, after wrapping/clamping. */
    private double controlTargetTicks;
    private double targetVelocityTicksPerSecond;
    private boolean targetTicksInitialized;
    private double integralPower;
    private double previousPositionError;
    private boolean previousPositionErrorValid;
    private double appliedPower;
    private double filteredMotorVelocityTicksPerSecond;
    private boolean motorVelocityInitialized;
    private double nearTargetStallSinceMs = -1;
    private double nearTargetStallSetpointTicks = Double.NaN;
    private boolean nearTargetStallHolding;

    // ---- diagnostics (read by logTo / events)
    private FlightRecorder rec;
    private int lastTicks;
    private double rawMotorVelocityTps;
    private long loopCount;
    private double dbgDtMs;
    private double dbgOdoUs, dbgVisionUs, dbgControlUs, dbgUpdateUs;
    private boolean newFrameThisLoop;
    private boolean odometryEverRead;
    private final java.util.ArrayDeque<double[]> poseHistory = new java.util.ArrayDeque<>();
    private double odoReadUs;
    private int odoStatusOrdinal;
    private long odoStatusRejects, odoJumpRejects;
    private int odoConsecutiveJumps;
    private double lastGoodOdoMs = -1;
    private double lastGoodX = Double.NaN, lastGoodY = Double.NaN, lastGoodReadMs;
    private int lastOdoStatusForEvents = 1;
    private long poseHistoryMisses;
    // fusion
    private boolean fusValid;
    private double fusX, fusY;
    private int fusFrames;
    private double fusMeasX = Double.NaN, fusMeasY = Double.NaN;
    private double fusResidDeg = Double.NaN, fusResidIn = Double.NaN;
    private double fusBearingDeg = Double.NaN, fusDistanceIn = Double.NaN;
    private double fusTargetTicks = Double.NaN;
    private double fusRateDps;
    private double fusPrevBearingDeg = Double.NaN, fusPrevBearingMs = -1;
    private long fusReseeds;
    private double fusPendingX = Double.NaN, fusPendingY = Double.NaN;
    private double fusPendingLastMs = -1;
    private int fusPendingFrames;
    private double fusMinusMeasTicks = Double.NaN;
    private double lastOdoReadMs;
    private double lastWrittenPower = Double.NaN;
    private double stopCandidateSinceMs = -1, moveCandidateSinceMs = -1;
    private boolean targetClamped;
    private boolean targetWrapped;
    /** Selected soft-stop exit while the target is inside the unreachable rear sector. */
    private double blindSectorRouteTicks = Double.NaN;
    private int pendingClusterFirstId = -1;
    private int pendingClusterFrames;
    private double pendingClusterLastFrameTimestamp = Double.NaN;
    private long framesTotal;
    private double lastFrameWallMs = -1;
    private double frameIntervalMs;
    private double dbgStaleness, dbgCaptureLat, dbgTargetingLat, dbgPipelineLat;
    private double dbgRawTxDeg, dbgTicksAtCapture, dbgTurretMoveDeg, dbgGain, dbgMeasuredTargetTicks;
    private double dbgRawBearingDeg, dbgCalibratedBearingDeg;
    private double dbgRequestedTargetTicks;
    private double dbgP, dbgD, dbgKV, dbgI, dbgStiction, dbgScale, dbgRawPower;
    private double dbgDesiredPower;
    private boolean dbgSaturated;
    private double robotSpeedInPerSec;
    private boolean robotStopped = true;
    private double stoppedSinceMs = -1;
    private double autoAimOnMs = -1;
    private double readyAfterStopMs = Double.NaN;
    private double readyAfterEnableMs = Double.NaN;
    private boolean readyReported;
    private boolean lastTargetVisibleForEvents;
    private boolean lastOdoTrackingForEvents;
    private boolean lastAtLimitForEvents;
    private boolean lastClampedForEvents;
    private boolean lastWrappedForEvents;
    private double lastBatteryV = Double.NaN;
    private double lastCurrentA = Double.NaN;
    private double lastSlowSampleMs = -1000;
    private double lastBatterySampleMs = -1000;
    private double indicatorPosition = Double.NaN;

    public turretSubsystem(Hardware robot) {
        this.robot = robot;
        turretMotor = robot.turret;
        // Never carry a stale command into this subsystem's initialization.
        turretMotor.setPower(0);
        robot.limelight.pipelineSwitch(APRILTAG_PIPELINE);
        robot.limelight.setPollRateHz(100);
        robot.limelight.start();
        writeIndicatorPosition(LED_MANUAL_POSITION);
        clock.reset();
    }

    public void setAutoAimEnabled(boolean enabled) {
        if (autoAimEnabled == enabled) return;
        autoAimEnabled = enabled;
        autoAimOnMs = enabled ? clock.milliseconds() : -1;
        readyAfterEnableMs = Double.NaN;
        readyAfterStopMs = Double.NaN;
        readyReported = false;
        event("AUTOAIM " + (enabled ? "ON" : "OFF") + " ticks=" + lastTicks
                + " visible=" + targetVisible + " robotStopped=" + robotStopped);
        targetTicksInitialized = false;
        integralPower = 0;
        previousPositionErrorValid = false;
        blindSectorRouteTicks = Double.NaN;
        resetNearTargetStallSafety();
        stableAimFrames = 0;
        aimReady = false;
        resetMeasurementHistory(true);
    }

    public void setManualPower(double power) {
        manualPower = clamp(power, -MAX_TURRET_POWER, MAX_TURRET_POWER);
    }

    public void setAlliance(Alliance alliance) {
        if (alliance == null || this.alliance == alliance) return;
        this.alliance = alliance;
        selectedClusterFirstId = -1;
        selectedClusterRollDeg = Double.NaN;
        resetSideAim();
        selectedTagId = -1;
        targetVisible = false;
        targetTicksInitialized = false;
        fusValid = false;
        blindSectorRouteTicks = Double.NaN;
        resetNearTargetStallSafety();
        clearFusionPending();
        clearPendingCluster();
        resetMeasurementHistory(false);
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
        lastWrittenPower = 0;
        robot.setTurretCurrentPositionAsZero();
        targetTicks = 0;
        controlTargetTicks = 0;
        targetVelocityTicksPerSecond = 0;
        targetTicksInitialized = false;
        integralPower = 0;
        previousPositionErrorValid = false;
        blindSectorRouteTicks = Double.NaN;
        resetNearTargetStallSafety();
        appliedPower = 0;
        manualPower = 0;
        encoderHistory.clear();
        motorVelocityInitialized = false;
        stableAimFrames = 0;
        aimReady = false;
    }

    /** Call once per OpMode loop. */
    public void update(double dtSeconds) {
        long updateStartNs = System.nanoTime();
        loopCount++;
        dbgDtMs = dtSeconds * 1000.0;
        double dt = clamp(dtSeconds, 0.001, 0.100);
        double nowMs = clock.milliseconds();
        int currentTicks = robot.getTurretPositionTicks();
        lastTicks = currentTicks;
        updateFilteredMotorVelocity();

        long t0 = System.nanoTime();
        updateOdometry();
        if (lastGoodOdoMs < 0 || nowMs - lastGoodOdoMs > ODO_STALE_MS) {
            odometryPoseValid = false;
            odometryRateValid = false;
        }
        long t1 = System.nanoTime();
        updateStoppedState(nowMs);
        saveEncoderSample(nowMs, currentTicks);
        newFrameThisLoop = false;
        updateVision(nowMs, currentTicks);
        updateFusionPrediction();
        odometryTrackingActive = !targetVisible && ODOMETRY_FUSION_ENABLED
                && fusValid && odometryPoseValid && Double.isFinite(fusTargetTicks);
        long t2 = System.nanoTime();

        if (autoAimEnabled) {
            applyAutomaticControl(dt, nowMs, currentTicks);
        } else {
            applyManualControl(currentTicks);
        }
        updateReadyState(nowMs, currentTicks);
        updateIndicator();
        emitTransitionEvents(nowMs, currentTicks);
        long t3 = System.nanoTime();

        dbgOdoUs = (t1 - t0) / 1000.0;
        dbgVisionUs = (t2 - t1) / 1000.0;
        dbgControlUs = (t3 - t2) / 1000.0;
        dbgUpdateUs = (t3 - updateStartNs) / 1000.0;
    }

    /** Stopped/moving state from Pinpoint with hysteresis + debounce, used to time "stop -> ready". */
    private void updateStoppedState(double nowMs) {
        if (!odometryRateValid) return;
        robotSpeedInPerSec = Math.hypot(robotForwardVelocityInPerSec, robotLeftVelocityInPerSec);
        double hr = Math.abs(Math.toDegrees(robotHeadingRateRadPerSec));
        boolean stillNow = robotSpeedInPerSec < STOPPED_SPEED_IN_PER_S && hr < STOPPED_HEADING_DEG_PER_S;
        boolean movingNow = robotSpeedInPerSec > MOVING_SPEED_IN_PER_S || hr > MOVING_HEADING_DEG_PER_S;

        stopCandidateSinceMs = stillNow ? (stopCandidateSinceMs < 0 ? nowMs : stopCandidateSinceMs) : -1;
        moveCandidateSinceMs = movingNow ? (moveCandidateSinceMs < 0 ? nowMs : moveCandidateSinceMs) : -1;

        if (!robotStopped && stopCandidateSinceMs >= 0
                && nowMs - stopCandidateSinceMs >= STOP_DEBOUNCE_MS) {
            robotStopped = true;
            // Credit the stop to when it really began, not after the debounce.
            stoppedSinceMs = stopCandidateSinceMs;
            readyAfterStopMs = Double.NaN;
            readyReported = false;
            event(String.format(java.util.Locale.US,
                    "ROBOT_STOPPED ticks=%d visible=%s ready=%s visErr=%+.2f",
                    lastTicks, targetVisible, aimReady, lastVisionErrorDeg));
        } else if (robotStopped && moveCandidateSinceMs >= 0
                && nowMs - moveCandidateSinceMs >= STOP_DEBOUNCE_MS) {
            robotStopped = false;
            stoppedSinceMs = -1;
            event(String.format(java.util.Locale.US,
                    "ROBOT_MOVING speed=%.1f in/s hdgRate=%.1f", robotSpeedInPerSec, hr));
        }
    }

    /**
     * Ready needs a stable small vision error AND (in auto mode) a turret that
     * is physically at its setpoint and nearly stationary.
     */
    private void updateReadyState(double nowMs, int currentTicks) {
        boolean targetKnown = targetVisible || odometryTrackingActive;
        boolean sourceStable = odometryTrackingActive
                || stableAimFrames >= AIM_STABLE_FRAMES;
        boolean settled = !autoAimEnabled
                || (Math.abs(controlTargetTicks - currentTicks) <= READY_POSITION_TOL_TICKS
                && Math.abs(filteredMotorVelocityTicksPerSecond) <= READY_MAX_VELOCITY_TPS);
        boolean wasReady = aimReady;
        aimReady = targetKnown && sourceStable && settled;
        if (aimReady && !wasReady && autoAimEnabled) {
            if (autoAimOnMs >= 0 && Double.isNaN(readyAfterEnableMs)) {
                readyAfterEnableMs = nowMs - autoAimOnMs;
            }
            if (robotStopped && stoppedSinceMs >= 0 && Double.isNaN(readyAfterStopMs)) {
                readyAfterStopMs = nowMs - stoppedSinceMs;
            }
            if (!readyReported) {
                readyReported = true;
                event(String.format(java.util.Locale.US,
                        "AIM_READY sinceEnable=%.0fms afterStop=%.0fms ticks=%d err=%+.2fdeg dist=%.1f",
                        autoAimOnMs >= 0 ? nowMs - autoAimOnMs : Double.NaN, readyAfterStopMs, currentTicks,
                        lastVisionErrorDeg, filteredDistanceIn));
            }
        } else if (!aimReady && wasReady) {
            event(String.format(java.util.Locale.US,
                    "AIM_LOST visible=%s visErr=%+.2f posErr=%+.1f vel=%+.0f",
                    targetVisible, lastVisionErrorDeg, controlTargetTicks - currentTicks,
                    filteredMotorVelocityTicksPerSecond));
        }
    }

    private void emitTransitionEvents(double nowMs, int currentTicks) {
        if (targetVisible != lastTargetVisibleForEvents) {
            lastTargetVisibleForEvents = targetVisible;
            event("TARGET_" + (targetVisible ? "ACQUIRED tags=" + visibleTagIds : "LOST"));
        }
        if (odometryTrackingActive != lastOdoTrackingForEvents) {
            lastOdoTrackingForEvents = odometryTrackingActive;
            event("ODOMETRY_TRACKING " + (odometryTrackingActive ? "ON" : "OFF"));
        }
        if (targetClamped != lastClampedForEvents) {
            lastClampedForEvents = targetClamped;
            event(targetClamped ? String.format(java.util.Locale.US,
                    "TARGET_OUT_OF_RANGE bearing=%.1f deg (turret limit +-%.0f deg)",
                    lastRobotBearingDeg, MAX_TURRET_TICKS / TICKS_PER_DEGREE)
                    : "TARGET_IN_RANGE");
        }
        if (targetWrapped != lastWrappedForEvents) {
            lastWrappedForEvents = targetWrapped;
            if (targetWrapped) {
                event(String.format(java.util.Locale.US,
                        "TARGET_WRAPPED requested=%+.1f ticks resolved=%+.1f ticks",
                        dbgRequestedTargetTicks, controlTargetTicks));
            }
        }
        boolean atLimit = currentTicks <= MIN_TURRET_TICKS || currentTicks >= MAX_TURRET_TICKS;
        if (atLimit != lastAtLimitForEvents) {
            lastAtLimitForEvents = atLimit;
            event("SOFT_LIMIT " + (atLimit ? "HIT ticks=" + currentTicks : "CLEARED"));
        }
        if (nowMs - lastSlowSampleMs >= 100) {
            lastSlowSampleMs = nowMs;
            try {
                lastCurrentA = robot.readTurretCurrentAmps();
                if (nowMs - lastBatterySampleMs >= 500) {
                    lastBatterySampleMs = nowMs;
                    lastBatteryV = robot.readBatteryVolts();
                }
            } catch (Throwable ignored) { }
        }
    }

    public void setRecorder(FlightRecorder recorder) {
        this.rec = recorder;
    }

    private void event(String message) {
        if (rec != null) rec.event(message);
    }

    /** Snapshot of every tunable, for the _config.txt file. */
    public static java.util.Map<String, String> configSnapshot() {
        java.util.Map<String, String> m = new java.util.LinkedHashMap<>();
        for (java.lang.reflect.Field f : turretSubsystem.class.getDeclaredFields()) {
            int mod = f.getModifiers();
            if (!java.lang.reflect.Modifier.isStatic(mod)) continue;
            try {
                f.setAccessible(true);
                Object v = f.get(null);
                m.put("turret." + f.getName(),
                        v instanceof double[] ? java.util.Arrays.toString((double[]) v) : String.valueOf(v));
            } catch (Throwable ignored) { }
        }
        return m;
    }

    /** Adds every turret diagnostic column to the current recorder row. */
    public void logTo(FlightRecorder r) {
        r.put("tur_loop", loopCount);
        r.put("tur_dt_ms", dbgDtMs);
        r.put("tur_upd_us", dbgUpdateUs);
        r.put("tur_odo_us", dbgOdoUs);
        r.put("tur_vision_us", dbgVisionUs);
        r.put("tur_ctrl_us", dbgControlUs);

        r.put("tur_auto", autoAimEnabled);
        r.put("tur_ticks", lastTicks);
        r.put("tur_raw_encoder_ticks", robot.getTurretRawEncoderTicks());
        r.put("tur_zero_offset_ticks", robot.getTurretEncoderZeroTicks());
        r.put("tur_target_ticks", targetTicks);
        r.put("tur_control_target_ticks", controlTargetTicks);
        r.put("tur_requested_target_ticks", dbgRequestedTargetTicks);
        r.put("tur_target_wrapped", targetWrapped);
        r.put("tur_pos_err", controlTargetTicks - lastTicks);
        r.put("tur_vel_filt", filteredMotorVelocityTicksPerSecond);
        r.put("tur_vel_raw", rawMotorVelocityTps);
        r.put("tur_tgt_vel", targetVelocityTicksPerSecond);
        r.put("tur_power", appliedPower);
        r.put("tur_manual_power", manualPower);
        r.put("tur_p", dbgP);
        r.put("tur_d", dbgD);
        r.put("tur_kv", dbgKV);
        r.put("tur_i", dbgI);
        r.put("tur_stiction", dbgStiction);
        r.put("tur_near_stall_hold", nearTargetStallHolding);
        r.put("tur_prox_scale", dbgScale);
        r.put("tur_raw_power", dbgRawPower);
        r.put("tur_desired_power", dbgDesiredPower);
        r.put("tur_saturated", dbgSaturated);
        r.put("tur_current_a", lastCurrentA);
        r.put("batt_v", lastBatteryV);
        r.put("tur_led_position", indicatorPosition);

        r.put("vis_visible", targetVisible);
        r.put("vis_tags", visibleTagCount);
        r.put("vis_new_frame", newFrameThisLoop);
        r.put("vis_frames_total", framesTotal);
        r.put("vis_frame_interval_ms", frameIntervalMs);
        r.put("vis_staleness_ms", dbgStaleness);
        r.put("vis_capture_lat_ms", dbgCaptureLat);
        r.put("vis_targeting_lat_ms", dbgTargetingLat);
        r.put("vis_total_lat_ms", lastLatencyMs);
        r.put("vis_raw_tx", dbgRawTxDeg);
        r.put("vis_filt_tx", filteredTxDeg);
        r.put("vis_err_deg", lastVisionErrorDeg);
        r.put("vis_ticks_at_capture", dbgTicksAtCapture);
        r.put("vis_turret_move_deg", dbgTurretMoveDeg);
        r.put("vis_gain", dbgGain);
        r.put("vis_measured_target_ticks", dbgMeasuredTargetTicks);
        r.put("aim_raw_bearing_deg", dbgRawBearingDeg);
        r.put("aim_calibrated_bearing_deg", dbgCalibratedBearingDeg);
        r.put("aim_camera_side_in", cameraSideOfClusterIn);
        r.put("aim_shooter_side_in", shooterSideOfClusterIn);
        r.put("aim_bank_offset_in", bankAimOffsetIn);
        r.put("aim_bank_correction_deg", bankAimCorrectionDeg);
        r.put("vis_stable_frames", stableAimFrames);
        r.put("vis_geometry_ok", clusterGeometryValid);
        r.put("vis_dist_in", filteredDistanceIn);
        r.put("vis_target_age_ms", Math.min(getTargetAgeMs(), 99999));

        r.put("aim_ready", aimReady);
        r.put("aim_ready_after_stop_ms", readyAfterStopMs);
        r.put("aim_ready_after_enable_ms", readyAfterEnableMs);

        r.put("odo_valid", odometryRateValid);
        r.put("odo_age_ms", clock.milliseconds() - lastOdoReadMs);
        r.put("odo_read_us", odoReadUs);
        r.put("odo_status", odoStatusOrdinal);
        r.put("odo_status_rejects", odoStatusRejects);
        r.put("odo_jump_rejects", odoJumpRejects);
        r.put("odo_hist_miss", poseHistoryMisses);
        r.put("fus_enabled", ODOMETRY_FUSION_ENABLED);
        r.put("fus_valid", fusValid);
        r.put("fus_frames", fusFrames);
        r.put("fus_reseeds", fusReseeds);
        r.put("fus_pending_frames", fusPendingFrames);
        r.put("cluster_pending_id", pendingClusterFirstId);
        r.put("cluster_pending_frames", pendingClusterFrames);
        r.put("fus_x", fusValid ? fusX : Double.NaN);
        r.put("fus_y", fusValid ? fusY : Double.NaN);
        r.put("fus_meas_x", fusMeasX);
        r.put("fus_meas_y", fusMeasY);
        r.put("fus_resid_deg", newFrameThisLoop ? fusResidDeg : Double.NaN);
        r.put("fus_resid_in", newFrameThisLoop ? fusResidIn : Double.NaN);
        r.put("fus_bearing_deg", fusBearingDeg);
        r.put("fus_dist_in", fusDistanceIn);
        r.put("fus_target_ticks", fusTargetTicks);
        r.put("fus_minus_meas_ticks", newFrameThisLoop ? fusMinusMeasTicks : Double.NaN);
        r.put("fus_minus_setpoint_ticks", fusValid ? fusTargetTicks - targetTicks : Double.NaN);
        r.put("fus_rate_dps", fusRateDps);
        r.put("tur_target_clamped", targetClamped);
        r.put("odo_x_in", robotFieldXIn);
        r.put("odo_y_in", robotFieldYIn);
        r.put("odo_heading_deg", Math.toDegrees(robotFieldHeadingRad));
        r.put("odo_vfwd", robotForwardVelocityInPerSec);
        r.put("odo_vleft", robotLeftVelocityInPerSec);
        r.put("odo_heading_rate", Math.toDegrees(robotHeadingRateRadPerSec));
        r.put("odo_speed", robotSpeedInPerSec);
        r.put("robot_stopped", robotStopped);
        r.put("bearing_deg", Double.isNaN(lastRobotBearingDeg) ? Double.NaN : lastRobotBearingDeg);
        r.put("odo_tracking", odometryTrackingActive);
    }

    private void updateVision(double nowMs, int currentTicks) {
        double targetAgeBeforeFrameMs = lastTargetSeenMs < 0
                ? Double.POSITIVE_INFINITY : nowMs - lastTargetSeenMs;
        LLResult result = robot.limelight.getLatestResult();
        boolean fresh = result != null && result.isValid()
                && result.getStaleness() >= 0
                && result.getStaleness() < MAX_RESULT_AGE_MS;

        List<LLResultTypes.FiducialResult> tags = fresh ? result.getFiducialResults() : null;
        updateVisibleTagText(tags);
        double resultTimestamp = result == null ? Double.NaN : result.getTimestamp();
        List<LLResultTypes.FiducialResult> clusterTags = selectClusterTags(
                tags, resultTimestamp);
        targetVisible = !clusterTags.isEmpty();
        visibleTagCount = clusterTags.size();

        if (result != null) {
            dbgStaleness = result.getStaleness();
            dbgCaptureLat = result.getCaptureLatency();
            dbgTargetingLat = result.getTargetingLatency();
        }

        if (!targetVisible) {
            clusterGeometryValid = false;
            aimReady = false;
            stableAimFrames = 0;
            return;
        }

        double frameTimestamp = resultTimestamp;
        if (frameTimestamp == lastFrameTimestamp) return;
        lastFrameTimestamp = frameTimestamp;
        lastTargetSeenMs = nowMs;
        newFrameThisLoop = true;
        framesTotal++;
        if (lastFrameWallMs >= 0) frameIntervalMs = nowMs - lastFrameWallMs;
        lastFrameWallMs = nowMs;

        ClusterAim clusterAim = calculateClusterAim(clusterTags);
        dbgRawTxDeg = clusterAim.txDeg;
        filteredTxDeg = addAndMedian(txHistory, txHistoryCount, clusterAim.txDeg);
        txHistoryCount = Math.min(txHistoryCount + 1, txHistory.length);
        updateDistance(clusterAim.averageTyDeg, selectedTagId,
                clusterAim.horizontalDistanceIn, targetAgeBeforeFrameMs);

        lastLatencyMs = result.getStaleness() + result.getCaptureLatency()
                + result.getTargetingLatency() + EXTRA_LATENCY_MS;
        double captureTimeMs = nowMs - lastLatencyMs;
        double ticksAtCapture = encoderTicksAt(captureTimeMs);
        dbgTicksAtCapture = ticksAtCapture;
        // A positive turret movement reduces a same-direction image error.
        double turretMovementDeg = LATENCY_COMPENSATION_ENABLED
                ? -(currentTicks - ticksAtCapture)
                  / (AIM_DIRECTION_SIGN * TICKS_PER_DEGREE) : 0;
        dbgTurretMoveDeg = turretMovementDeg;
        double horizontalErrorDeg = filteredTxDeg;

        // Robot-relative bearing = camera error + physical turret bearing.
        double bearingAtCaptureDeg = horizontalErrorDeg
                + ticksAtCapture / (AIM_DIRECTION_SIGN * TICKS_PER_DEGREE);
        double fusionDistanceIn = Double.isFinite(clusterAim.horizontalDistanceIn)
                ? clusterAim.horizontalDistanceIn : filteredDistanceIn;
        bankAimCorrectionDeg = bankAimCorrectionForDistance(fusionDistanceIn);
        double calibratedBearingAtCaptureDeg = calibratedAimBearingDeg(bearingAtCaptureDeg)
                + bankAimCorrectionDeg;
        dbgRawBearingDeg = bearingAtCaptureDeg;
        dbgCalibratedBearingDeg = calibratedBearingAtCaptureDeg;
        lastRobotBearingDeg = bearingAtCaptureDeg;
        // Fusion must remember the physical hive center, not whichever edge is
        // currently selected for the shot. The edge correction is added later.
        updateFusionFromFrame(nowMs, captureTimeMs, bearingAtCaptureDeg, fusionDistanceIn);

        // Convert the raw target bearing into the calibrated shooter bearing. Expressing
        // the error against the CURRENT turret position also performs latency compensation:
        // at scale 1.0 this is exactly horizontalError + turretMovement.
        double physicalTurretBearingNowDeg = currentTicks
                / (AIM_DIRECTION_SIGN * TICKS_PER_DEGREE);
        lastVisionErrorDeg = calibratedBearingAtCaptureDeg
                - physicalTurretBearingNowDeg;
        stableAimFrames = Math.abs(lastVisionErrorDeg) <= AIM_TOLERANCE_DEG
                ? stableAimFrames + 1 : 0;

        boolean fieldTargetUsable = ODOMETRY_FUSION_ENABLED && fusValid
                && odometryPoseValid && Double.isFinite(fusTargetTicks);
        double measuredTargetTicks = currentTicks
                + AIM_DIRECTION_SIGN * lastVisionErrorDeg * TICKS_PER_DEGREE;
        dbgMeasuredTargetTicks = measuredTargetTicks;
        fusMinusMeasTicks = Double.isNaN(fusTargetTicks)
                ? Double.NaN : fusTargetTicks - measuredTargetTicks;
        if (autoAimEnabled && !fieldTargetUsable) {
            if (!targetTicksInitialized) {
                targetTicks = measuredTargetTicks;
                targetTicksInitialized = true;
                dbgGain = 1.0;
            } else if (Math.abs(measuredTargetTicks - targetTicks) < VISION_DEADBAND_TICKS
                    && Math.abs(filteredMotorVelocityTicksPerSecond)
                    < VISION_DEADBAND_MAX_VELOCITY_TPS) {
                dbgGain = 0;
            } else {
                dbgGain = VISION_GAIN_MIN + (VISION_GAIN_MAX - VISION_GAIN_MIN)
                        * clamp(Math.abs(measuredTargetTicks - targetTicks)
                        / (VISION_GAIN_FULL_ERROR_DEG * TICKS_PER_DEGREE), 0, 1);
                targetTicks = (1.0 - dbgGain) * targetTicks + dbgGain * measuredTargetTicks;
            }
        } else {
            dbgGain = 0;
        }
    }

    private void updateDistance(
            double targetYDegrees, int tagId, double clusterDistanceIn,
            double targetAgeBeforeFrameMs) {
        double rawDistanceIn = clusterDistanceIn;
        if (!Double.isFinite(rawDistanceIn)) {
            rawDistanceIn = estimateTagFloorForwardFromTy(targetYDegrees, tagId);
        }
        if (rawDistanceIn <= 0 || rawDistanceIn >= MAX_DISTANCE_IN) return;

        // If vision was absent while the robot crossed the field, the old close
        // samples are no longer relevant. Re-seed from the newly reacquired CELL
        // instead of letting the stationary outlier guard lock in the old 2900-RPM range.
        if (targetAgeBeforeFrameMs > HOLD_AFTER_TARGET_LOSS_MS) {
            distanceHistoryCount = 0;
            distanceInitialized = false;
        }

        // Reject a bad pose/vertical-angle result before it contaminates the median history.
        // Cluster switches reset distanceInitialized, so a real jump to the other CELL still
        // seeds immediately instead of being compared with the previous CELL's range.
        double distanceOutlierLimitIn = robotStopped
                ? STATIONARY_DISTANCE_OUTLIER_IN : MOVING_DISTANCE_OUTLIER_IN;
        if (distanceInitialized
                && Math.abs(rawDistanceIn - filteredDistanceIn) > distanceOutlierLimitIn) {
            return;
        }

        double medianDistance = addAndMedian(
                distanceHistory, distanceHistoryCount, rawDistanceIn);
        distanceHistoryCount = Math.min(distanceHistoryCount + 1, distanceHistory.length);

        if (!distanceInitialized) {
            filteredDistanceIn = medianDistance;
            distanceInitialized = true;
        } else {
            filteredDistanceIn += DISTANCE_FILTER_ALPHA
                    * (medianDistance - filteredDistanceIn);
        }
    }

    /** Reads and owns the odometry state used by turret prediction/tracking. */
    private void updateOdometry() {
        if (robot.odo == null) return;
        if (odometryEverRead && (loopCount % ODOMETRY_READ_EVERY_N_LOOPS) != 0) return;
        odometryEverRead = true;
        double readStartMs = clock.milliseconds();
        lastOdoReadMs = readStartMs;
        robot.odo.update();
        odoReadUs = (clock.milliseconds() - readStartMs) * 1000.0;

        GoBildaPinpointDriver.DeviceStatus status = robot.odo.getDeviceStatus();
        odoStatusOrdinal = status.ordinal();
        if (odoStatusOrdinal != lastOdoStatusForEvents) {
            lastOdoStatusForEvents = odoStatusOrdinal;
            event("ODO_STATUS " + status);
        }
        if (status != GoBildaPinpointDriver.DeviceStatus.READY) {
            odoStatusRejects++;
            return;                       // keep the last good pose; staleness is checked in update()
        }
        double rawX = robot.odo.getPosX(DistanceUnit.INCH);
        double rawY = robot.odo.getPosY(DistanceUnit.INCH);
        if (!Double.isNaN(lastGoodX) && Double.isFinite(rawX) && Double.isFinite(rawY)) {
            double gapS = Math.max((readStartMs - lastGoodReadMs) / 1000.0, 0.001);
            double allowed = ODO_JUMP_SLACK_IN + ODO_MAX_SPEED_IN_PER_S * gapS;
            double jump = Math.hypot(rawX - lastGoodX, rawY - lastGoodY);
            if (jump > allowed && odoConsecutiveJumps < ODO_RESYNC_AFTER_JUMPS) {
                odoConsecutiveJumps++;
                odoJumpRejects++;
                if (odoJumpRejects <= 25) {
                    event(String.format(java.util.Locale.US,
                            "ODO_JUMP_REJECTED %.1f in in %.0f ms (allowed %.1f)", jump, gapS * 1000, allowed));
                }
                return;
            }
        }
        odoConsecutiveJumps = 0;
        lastGoodX = rawX;
        lastGoodY = rawY;
        lastGoodReadMs = readStartMs;
        lastGoodOdoMs = readStartMs;

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
            double sampleMs = readStartMs + (clock.milliseconds() - readStartMs) / 2.0;
            poseHistory.addLast(new double[]{sampleMs, fieldX, fieldY, headingRad});
            while (!poseHistory.isEmpty()
                    && sampleMs - poseHistory.peekFirst()[0] > POSE_HISTORY_MS) {
                poseHistory.removeFirst();
            }
        }
        if (!odometryRateValid) return;

        // Pinpoint X/Y velocity is already robot-relative: X forward, Y left.
        robotForwardVelocityInPerSec = ODOMETRY_FORWARD_SIGN * forwardVelocity;
        robotLeftVelocityInPerSec = ODOMETRY_LEFT_SIGN * leftVelocity;
        robotHeadingRateRadPerSec = ODOMETRY_HEADING_SIGN
                * Math.toRadians(headingRateDegPerSec);
    }

    /** Interpolated Pinpoint pose {x, y, heading} at a past time, or null if not covered. */
    private double[] poseAt(double timeMs) {
        double[] prev = null;
        for (double[] s : poseHistory) {
            if (s[0] >= timeMs) {
                if (prev == null) return null;      // older than our history
                double span = Math.max(s[0] - prev[0], 0.001);
                double f = (timeMs - prev[0]) / span;
                double dh = Math.atan2(Math.sin(s[3] - prev[3]), Math.cos(s[3] - prev[3]));
                return new double[]{prev[1] + f * (s[1] - prev[1]),
                        prev[2] + f * (s[2] - prev[2]), prev[3] + f * dh};
            }
            prev = s;
        }
        if (prev != null && timeMs - prev[0] < 40) {   // newer than the last sample: hold it
            return new double[]{prev[1], prev[2], prev[3]};
        }
        return null;
    }

    /** Field position of the turret axis for a given robot pose. */
    private static double[] pivotOf(double[] pose) {
        double c = Math.cos(pose[2]), s = Math.sin(pose[2]);
        return new double[]{
                pose[0] + OdometryConfig.TURRET_PIVOT_FORWARD_IN * c - OdometryConfig.TURRET_PIVOT_LEFT_IN * s,
                pose[1] + OdometryConfig.TURRET_PIVOT_FORWARD_IN * s + OdometryConfig.TURRET_PIVOT_LEFT_IN * c,
                pose[2]};
    }

    /** Field point seen from a pose at this camera bearing (code convention) and range. */
    private static double[] fieldPointFromBearing(double[] pose, double bearingDeg, double distIn) {
        double[] p = pivotOf(pose);
        double m = Math.toRadians(ODOMETRY_BEARING_SIGN * bearingDeg);
        double fwd = distIn * Math.cos(m), left = distIn * Math.sin(m);
        double c = Math.cos(p[2]), s = Math.sin(p[2]);
        return new double[]{p[0] + fwd * c - left * s, p[1] + fwd * s + left * c};
    }

    /** {bearingDeg, distanceIn} from a pose to a field point (inverse of the above). */
    private static double[] bearingDistanceTo(double[] pose, double tx, double ty) {
        double[] p = pivotOf(pose);
        double dx = tx - p[0], dy = ty - p[1];
        double c = Math.cos(p[2]), s = Math.sin(p[2]);
        double fwd = dx * c + dy * s, left = -dx * s + dy * c;
        return new double[]{ODOMETRY_BEARING_SIGN * Math.toDegrees(Math.atan2(left, fwd)),
                Math.hypot(fwd, left)};
    }

    private static double wrapDeg(double d) {
        return ((d + 540.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    /**
     * Symmetric left/right calibration with its wrap discontinuity inside the
     * turret's unreachable sector. That makes -170 and +190 degrees receive the
     * same correction instead of producing different targets for the same bearing.
     */
    private static double calibratedAimBearingDeg(double rawBearingDeg) {
        double minBearingDeg = MIN_TURRET_TICKS
                / (AIM_DIRECTION_SIGN * TICKS_PER_DEGREE);
        double maxBearingDeg = MAX_TURRET_TICKS
                / (AIM_DIRECTION_SIGN * TICKS_PER_DEGREE);
        double reachableCenterDeg = (minBearingDeg + maxBearingDeg) / 2.0;
        double canonicalBearingDeg = rawBearingDeg + 360.0
                * Math.rint((reachableCenterDeg - rawBearingDeg) / 360.0);
        return AIM_ANGLE_SCALE * canonicalBearingDeg;
    }

    /** Refines the field-anchored target with one camera frame (pose at capture time). */
    private void updateFusionFromFrame(
            double nowMs, double captureTimeMs, double bearingDeg, double distIn) {
        double[] pose = poseAt(captureTimeMs);
        if (pose == null || !odometryPoseValid || !Double.isFinite(distIn)
                || distIn < 1.0 || distIn > MAX_DISTANCE_IN) {
            poseHistoryMisses++;
            return;
        }
        double[] meas = fieldPointFromBearing(pose, bearingDeg, distIn);
        fusMeasX = meas[0];
        fusMeasY = meas[1];
        if (fusValid) {
            double[] pred = bearingDistanceTo(pose, fusX, fusY);
            fusResidDeg = wrapDeg(bearingDeg - pred[0]);
            fusResidIn = Math.hypot(meas[0] - fusX, meas[1] - fusY);
            if (Math.abs(fusResidDeg) > FUSION_REJECT_DEG || fusResidIn > FUSION_REJECT_IN) {
                boolean pendingExpired = fusPendingLastMs < 0
                        || nowMs - fusPendingLastMs > FUSION_CONFIRM_MAX_GAP_MS;
                boolean matchesPending = !pendingExpired
                        && Double.isFinite(fusPendingX) && Double.isFinite(fusPendingY)
                        && Math.hypot(meas[0] - fusPendingX, meas[1] - fusPendingY)
                        <= FUSION_CONFIRM_MATCH_IN;
                if (!matchesPending) {
                    fusPendingX = meas[0];
                    fusPendingY = meas[1];
                    fusPendingFrames = 1;
                } else {
                    fusPendingFrames++;
                    double pendingGain = 1.0 / fusPendingFrames;
                    fusPendingX += pendingGain * (meas[0] - fusPendingX);
                    fusPendingY += pendingGain * (meas[1] - fusPendingY);
                }
                fusPendingLastMs = nowMs;
                if (fusPendingFrames < FUSION_RESEED_CONFIRM_FRAMES) {
                    if (fusPendingFrames == 1) {
                        event(String.format(java.util.Locale.US,
                                "FUSION_OUTLIER_HELD residDeg=%+.1f residIn=%.1f",
                                fusResidDeg, fusResidIn));
                    }
                    return;
                }
                fusReseeds++;
                event(String.format(java.util.Locale.US,
                        "FUSION_RESEED_CONFIRMED residDeg=%+.1f residIn=%.1f frames=%d speed=%.1f hdgRate=%.1f",
                        fusResidDeg, fusResidIn, fusPendingFrames, robotSpeedInPerSec,
                        Math.toDegrees(robotHeadingRateRadPerSec)));
                fusX = fusPendingX;
                fusY = fusPendingY;
                fusFrames = 1;
                clearFusionPending();
            } else {
                clearFusionPending();
                double gain = robotStopped
                        ? clamp(FUSION_STILL_GAIN_MIN + FUSION_STILL_GAIN_PER_DEG * Math.abs(fusResidDeg),
                        FUSION_STILL_GAIN_MIN, FUSION_MEASUREMENT_GAIN)
                        : FUSION_MEASUREMENT_GAIN;
                fusX += gain * (meas[0] - fusX);
                fusY += gain * (meas[1] - fusY);
                fusFrames++;
            }
        } else {
            fusX = meas[0];
            fusY = meas[1];
            fusFrames = 1;
            fusValid = true;
            fusResidDeg = Double.NaN;
            fusResidIn = Double.NaN;
            clearFusionPending();
            event(String.format(java.util.Locale.US,
                    "FUSION_SEED field=(%.1f, %.1f) dist=%.1f", fusX, fusY, distIn));
        }
    }

    private void clearFusionPending() {
        fusPendingX = Double.NaN;
        fusPendingY = Double.NaN;
        fusPendingLastMs = -1;
        fusPendingFrames = 0;
    }

    /** Every loop: where should the turret point RIGHT NOW according to odometry. */
    private void updateFusionPrediction() {
        // A bad/stale Pinpoint sample must pause field tracking, not erase the
        // field point. Tracking resumes automatically when odometry recovers.
        if (!fusValid || !odometryPoseValid) {
            fusBearingDeg = fusDistanceIn = fusTargetTicks = Double.NaN;
            fusPrevBearingMs = -1;
            fusRateDps = 0;
            fusMinusMeasTicks = Double.NaN;
            return;
        }
        double[] bd = bearingDistanceTo(
                new double[]{robotFieldXIn, robotFieldYIn, robotFieldHeadingRad}, fusX, fusY);
        fusBearingDeg = bd[0];
        fusDistanceIn = bd[1];
        if (!Double.isFinite(fusDistanceIn) || fusDistanceIn < 1.0) {
            // Preserve the field anchor in case odometry becomes sane again.
            fusBearingDeg = fusDistanceIn = fusTargetTicks = Double.NaN;
            fusPrevBearingMs = -1;
            fusRateDps = 0;
            return;
        }
        // A distance beyond MAX_DISTANCE_IN is not trustworthy for shooter RPM,
        // but its field-point bearing still keeps the turret pointed at the hive.
        bankAimCorrectionDeg = bankAimCorrectionForDistance(fusDistanceIn);
        fusTargetTicks = AIM_DIRECTION_SIGN
                * (calibratedAimBearingDeg(fusBearingDeg) + bankAimCorrectionDeg)
                * TICKS_PER_DEGREE;

        if (fusPrevBearingMs >= 0 && lastOdoReadMs > fusPrevBearingMs) {
            double dtS = (lastOdoReadMs - fusPrevBearingMs) / 1000.0;
            double rate = clamp(wrapDeg(fusBearingDeg - fusPrevBearingDeg) / dtS,
                    -MAX_TARGET_RATE_DEG_PER_SEC * 3, MAX_TARGET_RATE_DEG_PER_SEC * 3);
            fusRateDps += FUSION_RATE_FILTER_ALPHA * (rate - fusRateDps);
            if (robotStopped) fusRateDps = 0;       // no feedforward from odometry noise while still
            fusPrevBearingDeg = fusBearingDeg;
            fusPrevBearingMs = lastOdoReadMs;
        } else if (fusPrevBearingMs < 0) {
            fusPrevBearingDeg = fusBearingDeg;
            fusPrevBearingMs = lastOdoReadMs;
        }

    }

    private void applyAutomaticControl(double dt, double nowMs, int currentTicks) {
        if (!targetTicksInitialized) targetTicks = currentTicks;

        double targetAgeMs = lastTargetSeenMs < 0
                ? Double.POSITIVE_INFINITY : nowMs - lastTargetSeenMs;
        boolean fusionActive = ODOMETRY_FUSION_ENABLED && fusValid
                && Double.isFinite(fusTargetTicks) && odometryPoseValid;
        if (fusionActive) {
            boolean holdStill = robotStopped && targetTicksInitialized
                    && Math.abs(fusTargetTicks - targetTicks) < FUSION_STILL_DEADBAND_TICKS
                    && Math.abs(filteredMotorVelocityTicksPerSecond) < 40.0;
            if (!holdStill) targetTicks = fusTargetTicks;
            targetTicksInitialized = true;
            targetVelocityTicksPerSecond = robotStopped
                    ? 0 : AIM_DIRECTION_SIGN * fusRateDps * TICKS_PER_DEGREE;
        } else if (targetAgeMs > HOLD_AFTER_TARGET_LOSS_MS) {
            targetVelocityTicksPerSecond = 0;
            integralPower = 0;
            // Hold the last setpoint if odometry briefly becomes unavailable.
            // The saved field target remains intact and resumes when pose data returns.
        } else {
            targetVelocityTicksPerSecond = 0;
        }

        dbgRequestedTargetTicks = targetTicks;
        controlTargetTicks = resolveTargetTicksWithinLimits(
                dbgRequestedTargetTicks, currentTicks,
                targetVelocityTicksPerSecond);
        targetWrapped = isEquivalentRevolution(
                dbgRequestedTargetTicks, controlTargetTicks)
                && Math.abs(controlTargetTicks - dbgRequestedTargetTicks) > 1e-6;
        targetClamped = !targetWrapped
                && Math.abs(controlTargetTicks - dbgRequestedTargetTicks) > 1e-6;
        if (targetClamped) targetVelocityTicksPerSecond = 0;
        double positionError = controlTargetTicks - currentTicks;
        // D damps turret motion only. Desired-motion feedforward is the separate
        // kV term, so changing D during tuning cannot also add target velocity.
        double velocityError = -filteredMotorVelocityTicksPerSecond;

        // Do not carry stored torque through the setpoint; that creates a
        // left-right limit cycle even when the vision target is steady.
        if (previousPositionErrorValid
                && positionError * previousPositionError < 0) {
            integralPower = 0;
        }
        previousPositionError = positionError;
        previousPositionErrorValid = true;

        dbgP = dbgD = dbgKV = dbgStiction = dbgRawPower = dbgDesiredPower = 0;
        dbgScale = 1;
        dbgSaturated = false;
        boolean stallSafetyHolding = updateNearTargetStallSafety(positionError, nowMs);
        if ((Math.abs(positionError) <= POSITION_TOLERANCE_TICKS
                && Math.abs(targetVelocityTicksPerSecond) < 2.0)
                || stallSafetyHolding) {
            appliedPower = 0;
            integralPower = 0;
        } else {
            if (POSITION_KI > 0 && Math.abs(positionError) <= INTEGRAL_ZONE_TICKS) {
                integralPower = clamp(
                        integralPower + POSITION_KI * positionError * dt,
                        -MAX_INTEGRAL_POWER, MAX_INTEGRAL_POWER);
            } else {
                integralPower = 0;
            }
            double staticFriction = STATIC_FRICTION_POWER == 0 ? 0
                    : Math.copySign(STATIC_FRICTION_POWER, positionError);
            double controllerPower =
                    POSITION_KP * positionError
                            + VELOCITY_KD * velocityError
                            + VELOCITY_KV * targetVelocityTicksPerSecond
                            + integralPower + staticFriction;

            dbgP = POSITION_KP * positionError;
            dbgD = VELOCITY_KD * velocityError;
            dbgKV = VELOCITY_KV * targetVelocityTicksPerSecond;
            dbgStiction = staticFriction;
            dbgRawPower = controllerPower;
            appliedPower = MOTOR_DIRECTION_SIGN * controllerPower;
        }
        dbgI = integralPower;
        dbgDesiredPower = appliedPower;
        dbgSaturated = Math.abs(appliedPower) > MAX_TURRET_POWER;
        appliedPower = limitAtSoftStops(
                clamp(appliedPower, -MAX_TURRET_POWER, MAX_TURRET_POWER), currentTicks);
        writePower(appliedPower);
    }

    /** Stops a near-target mechanical stall, but retries when the requested aim moves. */
    private boolean updateNearTargetStallSafety(double positionError, double nowMs) {
        double absError = Math.abs(positionError);
        if (nearTargetStallHolding) {
            boolean setpointMoved = Math.abs(
                    controlTargetTicks - nearTargetStallSetpointTicks)
                    >= NEAR_TARGET_STALL_RETRY_DELTA_TICKS;
            if (!setpointMoved && absError > POSITION_TOLERANCE_TICKS
                    && absError <= NEAR_TARGET_STALL_WINDOW_TICKS) {
                return true;
            }
            resetNearTargetStallSafety();
        }

        boolean nearlyStalled = absError > POSITION_TOLERANCE_TICKS
                && absError <= NEAR_TARGET_STALL_WINDOW_TICKS
                && Math.abs(targetVelocityTicksPerSecond) < 2.0
                && Math.abs(filteredMotorVelocityTicksPerSecond)
                <= NEAR_TARGET_STALL_MAX_VELOCITY_TPS;
        if (!nearlyStalled) {
            nearTargetStallSinceMs = -1;
            nearTargetStallSetpointTicks = Double.NaN;
            return false;
        }
        if (nearTargetStallSinceMs < 0
                || Math.abs(controlTargetTicks - nearTargetStallSetpointTicks)
                >= NEAR_TARGET_STALL_RETRY_DELTA_TICKS) {
            nearTargetStallSinceMs = nowMs;
            nearTargetStallSetpointTicks = controlTargetTicks;
            return false;
        }
        if (nowMs - nearTargetStallSinceMs < NEAR_TARGET_STALL_TIMEOUT_MS) {
            return false;
        }

        nearTargetStallHolding = true;
        integralPower = 0;
        event(String.format(java.util.Locale.US,
                "NEAR_TARGET_STALL_HOLD error=%+.1f target=%.1f ticks=%d",
                positionError, controlTargetTicks, lastTicks));
        return true;
    }

    private void resetNearTargetStallSafety() {
        nearTargetStallSinceMs = -1;
        nearTargetStallSetpointTicks = Double.NaN;
        nearTargetStallHolding = false;
    }

    private void updateFilteredMotorVelocity() {
        double rawVelocity = turretMotor.getVelocity();
        rawMotorVelocityTps = rawVelocity;
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
        previousPositionErrorValid = false;
        blindSectorRouteTicks = Double.NaN;
        resetNearTargetStallSafety();
        dbgP = dbgD = dbgKV = dbgStiction = dbgRawPower = dbgI = 0;
        dbgDesiredPower = manualPower;
        dbgScale = 1;
        appliedPower = limitAtSoftStops(manualPower, currentTicks);
        targetClamped = false;
        targetWrapped = false;
        dbgRequestedTargetTicks = currentTicks;
        controlTargetTicks = currentTicks;
        writePower(appliedPower);
    }

    /** Every setPower is a ~1-2 ms hub command; skip it when nothing meaningful changed. */
    private void writePower(double power) {
        if (Double.isNaN(lastWrittenPower)
                || Math.abs(power - lastWrittenPower) > 0.004
                || (power == 0) != (lastWrittenPower == 0)) {
            turretMotor.setPower(power);
            lastWrittenPower = power;
        }
    }

    private void updateIndicator() {
        double position;
        if (!autoAimEnabled) {
            position = LED_MANUAL_POSITION;
        } else if (aimReady && targetVisible) {
            // Green is reserved for a stable, currently visible Limelight target.
            position = LED_VISION_LOCK_POSITION;
        } else if (aimReady && odometryTrackingActive) {
            // The turret is settled, but the target is only being carried by odometry.
            position = LED_ODOMETRY_POSITION;
        } else {
            position = LED_ALIGNING_POSITION;
        }
        writeIndicatorPosition(position);
    }

    private void writeIndicatorPosition(double position) {
        if (robot.shooterLed.isPwmEnabled()
                && Double.isFinite(indicatorPosition)
                && Math.abs(position - indicatorPosition) < LED_MIN_POSITION_CHANGE) {
            return;
        }
        indicatorPosition = position;
        robot.shooterLed.setPosition(position);
        robot.shooterLed.setPwmEnable();
    }

    private double limitAtSoftStops(double power, int currentTicks) {
        double tickDirection = MOTOR_DIRECTION_SIGN * power;
        if (currentTicks >= MAX_TURRET_TICKS && tickDirection > 0) return 0;
        if (currentTicks <= MIN_TURRET_TICKS && tickDirection < 0) return 0;
        return power;
    }

    /**
     * Maps an angular target to the closest equivalent encoder target inside
     * the physical soft stops. If the angle is in the blind sector, returns
     * the appropriate stop until an equivalent angle becomes reachable.
     */
    private double resolveTargetTicksWithinLimits(
            double requestedTicks, double currentTicks,
            double requestedVelocityTicksPerSecond) {
        if (!Double.isFinite(requestedTicks)) {
            blindSectorRouteTicks = Double.NaN;
            return clampStatic(currentTicks, MIN_TURRET_TICKS, MAX_TURRET_TICKS);
        }
        if (requestedTicks >= MIN_TURRET_TICKS
                && requestedTicks <= MAX_TURRET_TICKS) {
            blindSectorRouteTicks = Double.NaN;
            return requestedTicks;
        }

        double revolution = Math.abs(TURRET_TICKS_PER_REVOLUTION);
        if (revolution < 1e-6) {
            blindSectorRouteTicks = Double.NaN;
            return clampStatic(requestedTicks, MIN_TURRET_TICKS, MAX_TURRET_TICKS);
        }

        long firstTurn = (long) Math.ceil(
                (MIN_TURRET_TICKS - requestedTicks) / revolution);
        long lastTurn = (long) Math.floor(
                (MAX_TURRET_TICKS - requestedTicks) / revolution);
        if (firstTurn <= lastTurn) {
            blindSectorRouteTicks = Double.NaN;
            double best = requestedTicks + firstTurn * revolution;
            double bestTravel = Math.abs(best - currentTicks);
            for (long turn = firstTurn + 1; turn <= lastTurn; turn++) {
                double candidate = requestedTicks + turn * revolution;
                double travel = Math.abs(candidate - currentTicks);
                if (travel < bestTravel) {
                    best = candidate;
                    bestTravel = travel;
                }
            }
            return best;
        }
        // Once a route through the unreachable rear sector is selected, keep
        // that route until an equivalent target is physically reachable again.
        // Without this latch, the filtered target rate can cross the routing
        // threshold and make the command flip rapidly between both soft stops.
        if (blindSectorRouteTicks == MIN_TURRET_TICKS
                || blindSectorRouteTicks == MAX_TURRET_TICKS) {
            return blindSectorRouteTicks;
        }
        // While the target moves through the unreachable rear sector, route
        // toward the boundary where it will become reachable again. This starts
        // a necessary wrap immediately instead of waiting until the sector midpoint.
        if (requestedVelocityTicksPerSecond > WRAP_ROUTE_MIN_VELOCITY_TPS) {
            blindSectorRouteTicks = MIN_TURRET_TICKS;
            return blindSectorRouteTicks;
        }
        if (requestedVelocityTicksPerSecond < -WRAP_ROUTE_MIN_VELOCITY_TPS) {
            blindSectorRouteTicks = MAX_TURRET_TICKS;
            return blindSectorRouteTicks;
        }
        // The requested angle is in the physical blind sector. Choose the soft
        // stop that is closest in ANGLE, not the one closest numerically. Near
        // the middle of that sector, stay on the side nearest the current turret
        // position so a little odometry/vision noise cannot command a full wrap.
        double distanceToMin = equivalentTickDistance(
                requestedTicks, MIN_TURRET_TICKS, revolution);
        double distanceToMax = equivalentTickDistance(
                requestedTicks, MAX_TURRET_TICKS, revolution);
        double switchHysteresisTicks = BLIND_SECTOR_SWITCH_HYSTERESIS_DEG
                * TICKS_PER_DEGREE;
        if (Math.abs(distanceToMin - distanceToMax) <= switchHysteresisTicks) {
            blindSectorRouteTicks = Math.abs(currentTicks - MIN_TURRET_TICKS)
                    <= Math.abs(currentTicks - MAX_TURRET_TICKS)
                    ? MIN_TURRET_TICKS : MAX_TURRET_TICKS;
            return blindSectorRouteTicks;
        }
        blindSectorRouteTicks = distanceToMin < distanceToMax
                ? MIN_TURRET_TICKS : MAX_TURRET_TICKS;
        return blindSectorRouteTicks;
    }

    private static double equivalentTickDistance(
            double firstTicks, double secondTicks, double revolution) {
        double delta = (firstTicks - secondTicks) % revolution;
        if (delta > revolution / 2.0) delta -= revolution;
        if (delta < -revolution / 2.0) delta += revolution;
        return Math.abs(delta);
    }

    private static boolean isEquivalentRevolution(double firstTicks, double secondTicks) {
        double turns = (secondTicks - firstTicks) / TURRET_TICKS_PER_REVOLUTION;
        return Double.isFinite(turns) && Math.abs(turns - Math.rint(turns)) < 1e-6;
    }

    private static double clampStatic(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private List<LLResultTypes.FiducialResult> selectClusterTags(
            List<LLResultTypes.FiducialResult> tags, double frameTimestamp) {
        List<LLResultTypes.FiducialResult> empty = new ArrayList<>();
        if (tags == null || tags.isEmpty()) {
            selectedClusterRollDeg = Double.NaN;
            clearPendingCluster();
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
            clearPendingCluster();
            return empty;
        }

        if (selectedClusterFirstId != selected.firstId) {
            int previousClusterFirstId = selectedClusterFirstId;
            if (pendingClusterFirstId != selected.firstId) {
                pendingClusterFirstId = selected.firstId;
                pendingClusterFrames = 1;
                pendingClusterLastFrameTimestamp = frameTimestamp;
            } else if (frameTimestamp != pendingClusterLastFrameTimestamp) {
                pendingClusterFrames++;
                pendingClusterLastFrameTimestamp = frameTimestamp;
            }
            if (pendingClusterFrames < CLUSTER_SWITCH_CONFIRM_FRAMES) {
                selectedClusterRollDeg = Double.NaN;
                return empty;
            }
            selectedClusterFirstId = selected.firstId;
            resetSideAim();
            targetTicksInitialized = false;
            // Keep the previous field point until updateFusionFromFrame accepts
            // a consistent replacement. Clearing it here made the turret forget
            // the hive during the exact transition where memory matters most.
            clearFusionPending();
            resetMeasurementHistory(true);
            event(String.format(java.util.Locale.US,
                    "CELL_SWITCH old=%d new=%d memory=%s",
                    previousClusterFirstId, selected.firstId,
                    fusValid ? "PRESERVED" : "EMPTY"));
        }
        clearPendingCluster();
        selectedClusterRollDeg = selected.rollDeg;
        selectedTagId = Integer.MAX_VALUE;
        for (LLResultTypes.FiducialResult tag : selected.tags) {
            selectedTagId = Math.min(selectedTagId, tag.getFiducialId());
        }
        return selected.tags;
    }

    private void clearPendingCluster() {
        pendingClusterFirstId = -1;
        pendingClusterFrames = 0;
        pendingClusterLastFrameTimestamp = Double.NaN;
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
        double sumTy = 0;

        updateSideAim(tags);

        for (LLResultTypes.FiducialResult tag : tags) {
            double correctedTxDeg = tag.getTargetXDegrees()
                    - aimOffsetForTag(tag.getFiducialId());
            sumTanTx += Math.tan(Math.toRadians(correctedTxDeg));
            sumTy += tag.getTargetYDegrees();
        }

        double meanTanTx = sumTanTx / count;
        double averageTyDeg = sumTy / count;
        // Height + Ty gives a stable range source. Keep its raw value responsive for
        // RPM, but filter the copy used by horizontal parallax so Ty noise cannot
        // move the turret setpoint left and right while the robot is stationary.
        double measuredTagPlaneDepthIn = estimateOpticalDepthFromTy(
                averageTyDeg, selectedTagId);
        if (!Double.isFinite(measuredTagPlaneDepthIn)) {
            measuredTagPlaneDepthIn = lastClusterTagPlaneDepthIn;
        }

        double aimTagPlaneDepthIn = measuredTagPlaneDepthIn;
        if (Double.isFinite(measuredTagPlaneDepthIn)
                && Double.isFinite(lastClusterTagPlaneDepthIn)
                && Math.abs(measuredTagPlaneDepthIn - lastClusterTagPlaneDepthIn)
                <= AIM_DEPTH_RESEED_DELTA_IN) {
            double depthGain = robotStopped
                    ? AIM_DEPTH_FILTER_ALPHA_STILL : AIM_DEPTH_FILTER_ALPHA_MOVING;
            aimTagPlaneDepthIn = lastClusterTagPlaneDepthIn
                    + depthGain * (measuredTagPlaneDepthIn - lastClusterTagPlaneDepthIn);
        }
        double aimClusterLateralIn = Double.NaN;
        double measuredClusterLateralIn = Double.NaN;

        if (Double.isFinite(aimTagPlaneDepthIn)
                && Double.isFinite(measuredTagPlaneDepthIn)
                && aimTagPlaneDepthIn > CLUSTER_AIM_POINT_FORWARD_OF_TAGS_IN
                + CLUSTER_AIM_POINT_ABOVE_TAGS_IN + 1.0
                && measuredTagPlaneDepthIn > CLUSTER_AIM_POINT_FORWARD_OF_TAGS_IN
                + CLUSTER_AIM_POINT_ABOVE_TAGS_IN + 1.0) {
            double aimLateralSum = 0;
            double measuredLateralSum = 0;
            for (LLResultTypes.FiducialResult tag : tags) {
                double correctedTxDeg = tag.getTargetXDegrees()
                        - aimOffsetForTag(tag.getFiducialId());
                double tanTx = Math.tan(Math.toRadians(correctedTxDeg));
                double tagXIn = clusterXForTag(tag.getFiducialId());
                aimLateralSum += aimTagPlaneDepthIn * tanTx - tagXIn;
                measuredLateralSum += measuredTagPlaneDepthIn * tanTx - tagXIn;
            }
            aimClusterLateralIn = aimLateralSum / count;
            measuredClusterLateralIn = measuredLateralSum / count;
            lastClusterTagPlaneDepthIn = aimTagPlaneDepthIn;
        }

        clusterGeometryValid = Double.isFinite(aimClusterLateralIn)
                && Double.isFinite(measuredClusterLateralIn)
                && Double.isFinite(aimTagPlaneDepthIn)
                && Double.isFinite(measuredTagPlaneDepthIn);
        double centerTxDeg;
        double horizontalDistanceIn = Double.NaN;
        if (clusterGeometryValid) {
            // SDK cluster coordinates put each tag behind and below the CELL opening
            // origin. Project that field-space offset into the upward-pitched camera.
            double cameraPitchRad = Math.toRadians(CAMERA_UP_ANGLE_DEG);
            double tagToOriginDepthIn = CLUSTER_AIM_POINT_FORWARD_OF_TAGS_IN
                    * Math.cos(cameraPitchRad)
                    - CLUSTER_AIM_POINT_ABOVE_TAGS_IN * Math.sin(cameraPitchRad);
            double aimPointDepthIn = aimTagPlaneDepthIn - tagToOriginDepthIn;
            double measuredAimPointDepthIn = measuredTagPlaneDepthIn
                    - tagToOriginDepthIn;

            // The Limelight is left of the shooter. Convert the opening's lateral
            // position from the camera origin to the shooter origin before aiming.
            // This is a distance-based parallax correction, not a fixed angle.
            double aimShooterLateralIn = aimClusterLateralIn
                    - CAMERA_LEFT_OF_SHOOTER_IN;
            centerTxDeg = Math.toDegrees(
                    Math.atan2(aimShooterLateralIn, aimPointDepthIn));

            double tagVerticalOpticalIn = measuredTagPlaneDepthIn
                    * Math.tan(Math.toRadians(averageTyDeg));
            double aimPointVerticalOpticalIn = tagVerticalOpticalIn
                    + CLUSTER_AIM_POINT_FORWARD_OF_TAGS_IN * Math.sin(cameraPitchRad)
                    + CLUSTER_AIM_POINT_ABOVE_TAGS_IN * Math.cos(cameraPitchRad);
            double aimPointForwardIn = measuredAimPointDepthIn * Math.cos(cameraPitchRad)
                    - aimPointVerticalOpticalIn * Math.sin(cameraPitchRad);
            double measuredShooterLateralIn = measuredClusterLateralIn
                    - CAMERA_LEFT_OF_SHOOTER_IN;
            horizontalDistanceIn = Math.hypot(
                    aimPointForwardIn, measuredShooterLateralIn);
        } else {
            // Safe startup fallback if Limelight 3D is disabled and only one tag
            // has ever been visible. Multiple visible tags use their angular mean.
            centerTxDeg = Math.toDegrees(Math.atan(meanTanTx));
        }
        return new ClusterAim(centerTxDeg, averageTyDeg, horizontalDistanceIn);
    }

    /** Converts the selected lateral edge into an angle without moving the saved hive. */
    private double bankAimCorrectionForDistance(double distanceIn) {
        if (!Double.isFinite(distanceIn) || distanceIn < 1.0) return 0;
        return Math.toDegrees(Math.atan2(bankAimOffsetIn, distanceIn));
    }

    /**
     * Locates the camera left/right of the complete four-tag cluster. Each pose
     * is relative to one tag, so adding that tag's cluster X offset produces a
     * common opening-centered coordinate before the observations are averaged.
     */
    private void updateSideAim(List<LLResultTypes.FiducialResult> tags) {
        double cameraClusterXSumIn = 0;
        int samples = 0;
        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag.getCameraPoseTargetSpace() == null
                    || tag.getCameraPoseTargetSpace().getPosition() == null) continue;
            Position cameraPosition = tag.getCameraPoseTargetSpace().getPosition();
            double cameraXIn = DistanceUnit.INCH.fromUnit(
                    cameraPosition.unit, cameraPosition.x);
            if (!Double.isFinite(cameraXIn)) continue;
            cameraClusterXSumIn += cameraXIn + clusterXForTag(tag.getFiducialId());
            samples++;
        }
        if (samples == 0) return;

        double measuredSideIn = cameraClusterXSumIn / samples;
        if (!Double.isFinite(cameraSideOfClusterIn)) {
            cameraSideOfClusterIn = measuredSideIn;
        } else {
            cameraSideOfClusterIn += SIDE_POSITION_FILTER_ALPHA
                    * (measuredSideIn - cameraSideOfClusterIn);
        }

        // The Limelight is left of the shooter, so convert the camera's
        // target-space position into the shooter's position before deciding
        // which side/edge should be used.
        shooterSideOfClusterIn = cameraSideOfClusterIn + CAMERA_LEFT_OF_SHOOTER_IN;

        double rightSideFraction = clamp(
                (shooterSideOfClusterIn - SIDE_AIM_START_RIGHT_IN)
                        / Math.max(0.1,
                        SIDE_AIM_FULL_RIGHT_IN - SIDE_AIM_START_RIGHT_IN),
                0, 1);
        double requestedBankOffsetIn = -RIGHT_POSITION_AIM_LEFT_IN
                * rightSideFraction;
        bankAimOffsetIn += BANK_AIM_FILTER_ALPHA
                * (requestedBankOffsetIn - bankAimOffsetIn);
    }

    private void resetSideAim() {
        cameraSideOfClusterIn = Double.NaN;
        shooterSideOfClusterIn = Double.NaN;
        bankAimOffsetIn = 0;
        bankAimCorrectionDeg = 0;
    }

    private double estimateOpticalDepthFromTy(double targetYDegrees, int tagId) {
        double tagAboveCameraIn = tagHeightForId(tagId) - CAMERA_HEIGHT_IN;
        double floorForwardIn = estimateTagFloorForwardFromTy(targetYDegrees, tagId);
        if (!Double.isFinite(floorForwardIn)) return Double.NaN;

        // Convert the field-horizontal distance back to camera optical depth;
        // calculateClusterAim later projects the opening aim point into the field plane.
        double cameraPitchRad = Math.toRadians(CAMERA_UP_ANGLE_DEG);
        return floorForwardIn * Math.cos(cameraPitchRad)
                + tagAboveCameraIn * Math.sin(cameraPitchRad);
    }

    /** Horizontal camera-to-tag distance used only if full cluster geometry is unavailable. */
    private double estimateTagFloorForwardFromTy(double targetYDegrees, int tagId) {
        double tagAboveCameraIn = tagHeightForId(tagId) - CAMERA_HEIGHT_IN;
        double elevationAngleDeg = CAMERA_UP_ANGLE_DEG + targetYDegrees;
        if (!Double.isFinite(tagAboveCameraIn) || tagAboveCameraIn <= 0
                || !Double.isFinite(elevationAngleDeg)
                || elevationAngleDeg <= MIN_DISTANCE_ANGLE_DEG
                || elevationAngleDeg >= 89.0) {
            return Double.NaN;
        }
        return tagAboveCameraIn / Math.tan(Math.toRadians(elevationAngleDeg));
    }

    private double clusterXForTag(int id) {
        int index = indexOfTag(id);
        if (index < 0 || index >= CLUSTER_TAG_X_OFFSETS_IN.length) return 0;

        // These offsets use the Limelight Tx/image convention and are ordered by
        // increasing tag ID. Reversing the scoring-wall IDs made outer-tag-only
        // observations of the same CELL disagree by about 11 degrees.
        return CLUSTER_TAG_X_OFFSETS_IN[index];
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

    private void resetMeasurementHistory(boolean preserveFieldTarget) {
        lastFrameTimestamp = -1;
        txHistoryCount = 0;
        distanceHistoryCount = 0;
        distanceInitialized = false;
        lastRobotBearingDeg = Double.NaN;
        if (!preserveFieldTarget) fusValid = false;
        odometryTrackingActive = false;
        lastClusterTagPlaneDepthIn = Double.NaN;
        clusterGeometryValid = false;
        aimReady = false;
        stableAimFrames = 0;
    }

    public boolean hasTarget() { return targetVisible; }
    public boolean hasDistance() { return distanceInitialized; }
    public boolean hasFieldTargetMemory() { return fusValid; }
    public boolean isAimReady() { return aimReady; }
    public int getSelectedTagId() { return selectedTagId; }
    public Alliance getAlliance() { return alliance; }
    public int getSelectedClusterFirstId() { return selectedClusterFirstId; }
    public double getSelectedClusterRollDeg() { return selectedClusterRollDeg; }
    public double getCameraSideOfClusterIn() { return cameraSideOfClusterIn; }
    public double getShooterSideOfClusterIn() { return shooterSideOfClusterIn; }
    public double getBankAimOffsetIn() { return bankAimOffsetIn; }
    public double getBankAimCorrectionDeg() { return bankAimCorrectionDeg; }
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
    public boolean hasFusedDistance() {
        return ODOMETRY_FUSION_ENABLED && fusValid && odometryPoseValid
                && Double.isFinite(fusDistanceIn)
                && fusDistanceIn > 0 && fusDistanceIn <= MAX_DISTANCE_IN;
    }
    public double getFusedDistanceIn() { return fusDistanceIn; }
    /**
     * Stable range for the RPM table. Vision seeds/corrects the field target;
     * odometry updates its distance every loop and carries it through brief occlusion.
     */
    public double getShooterDistanceIn() {
        if (hasFusedDistance()
                && (!distanceInitialized
                || !targetVisible
                || Math.abs(fusDistanceIn - filteredDistanceIn) <= FUSION_REJECT_IN)) {
            return fusDistanceIn;
        }
        return filteredDistanceIn;
    }
    public double getRobotForwardVelocityInPerSec() { return robotForwardVelocityInPerSec; }
    public double getRobotLeftVelocityInPerSec() { return robotLeftVelocityInPerSec; }
    public double getRobotHeadingRateDegPerSec() {
        return Math.toDegrees(robotHeadingRateRadPerSec);
    }
    public boolean isOdometryRateValid() { return odometryRateValid; }
    public boolean isOdometryPoseValid() { return odometryPoseValid; }
    public boolean isOdometryTrackingActive() { return odometryTrackingActive; }
    public double getOdometryTrackedBearingDeg() { return fusBearingDeg; }
    public double getOdometryTrackedDistanceIn() { return fusDistanceIn; }
    public double getTargetAgeMs() {
        return lastTargetSeenMs < 0
                ? Double.POSITIVE_INFINITY : clock.milliseconds() - lastTargetSeenMs;
    }
    public double getLastVisionErrorDeg() { return lastVisionErrorDeg; }
    public double getLastLatencyMs() { return lastLatencyMs; }
    public int getCurrentTicks() { return lastTicks; }
    public int getRawEncoderTicks() { return robot.getTurretRawEncoderTicks(); }
    public int getEncoderZeroTicks() { return robot.getTurretEncoderZeroTicks(); }
    public double getTargetTicks() { return controlTargetTicks; }
    public double getPositionErrorTicks() {
        return controlTargetTicks - lastTicks;
    }
    public double getFilteredMotorVelocityTicksPerSecond() {
        return filteredMotorVelocityTicksPerSecond;
    }
    public double getIntegralPower() { return integralPower; }
    public double getReadyAfterStopMs() { return readyAfterStopMs; }
    public double getReadyAfterEnableMs() { return readyAfterEnableMs; }
    public double getUpdateMicros() { return dbgUpdateUs; }
    public double getRobotSpeedInPerSec() { return robotSpeedInPerSec; }
    public boolean isRobotStopped() { return robotStopped; }
    public double getAppliedPower() { return appliedPower; }
    public double getBatteryVolts() { return lastBatteryV; }

    public void stop() {
        turretMotor.setPower(0);
        lastWrittenPower = 0;
        integralPower = 0;
        previousPositionErrorValid = false;
        blindSectorRouteTicks = Double.NaN;
        resetNearTargetStallSafety();
        writeIndicatorPosition(LED_MANUAL_POSITION);
        robot.limelight.stop();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
