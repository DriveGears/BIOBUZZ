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
    public static final String TUNE_VERSION = "clean-v21-auto-field-handoff";

    /** Limelight AprilTag pipeline number. */
    public static final int APRILTAG_PIPELINE = 1;

    public enum Alliance { RED, BLUE }

    /** The one authoritative source currently allowed to command the turret. */
    public enum TrackingState {
        MANUAL,
        PAUSED,
        VISION,
        ODOMETRY,
        HOLD,
        FAULT,
        RESCAN,
        FIELD_HANDOFF
    }

    /** First AprilTag ID in each four-tag CELL cluster. */
    public static final int RED_SCORING_CLUSTER_FIRST_ID = 30;
    public static final int RED_AUDIENCE_CLUSTER_FIRST_ID = 34;
    public static final int BLUE_AUDIENCE_CLUSTER_FIRST_ID = 38;
    public static final int BLUE_SCORING_CLUSTER_FIRST_ID = 42;

    /**
     * Symmetric angular calibration. Values above 1.0 aim progressively farther
     * from robot-forward on both sides; camera-to-shooter parallax remains separate.
     * Keep at 1.0 until the turret PID and mechanics are verified.
     */
    public static double AIM_ANGLE_SCALE = 0.96;

    /**
     * Constant left/right calibration applied after AIM_ANGLE_SCALE.
     * Positive values move the requested target toward increasing turret ticks.
     * Use the turret test OpMode's live trim to find this value, then copy the
     * resulting total offset here for competition.
     */
    public static double AIM_OFFSET_DEG = 0.0;

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
    public static final int MAX_TURRET_TICKS = 470;
    /** A true relative-position overrun stops all power until the turret is safely re-zeroed. */
    public static final int HARD_LIMIT_OVERRUN_TICKS = 25;
    public static final double MAX_TURRET_POWER = 0.85;
    /** Slow camera-search power; normal vision/odometry tracking still uses MAX_TURRET_POWER. */
    public static double RESCAN_MAX_POWER = 0.22;
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
    public static double CAMERA_LEFT_OF_SHOOTER_IN = 3.5;

    /**
     * Symmetric bank aim used to reduce backspin. A shooter on the right aims
     * left of center; a shooter on the left aims right. Even at center the aim
     * stays BANK_AIM_CENTER_OFFSET_IN to one side. The chosen side is latched
     * until the shooter crosses the opposite hysteresis boundary and the hold
     * timer has expired, preventing noisy center measurements from side-chattering.
     */
    public static double BANK_AIM_CENTER_OFFSET_IN = 0.0;
    public static double BANK_AIM_MAX_OFFSET_IN = 0.0;
    public static double BANK_AIM_FULL_SIDE_IN = 3.0;//
    public static double BANK_AIM_SWITCH_HYSTERESIS_IN = 3.0;
    public static double BANK_AIM_SWITCH_HOLD_MS = 750.0;
    /** One filter for side-position geometry; the bank command itself is not filtered again. */
    private static final double SIDE_AIM_FILTER_ALPHA = 0.25;

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
     * The complete motor tuning surface: position P, velocity damping D, and
     * breakaway power. There is intentionally no integral or velocity feedforward.
     */
    public static double POSITION_KP = 0.040;
    public static double VELOCITY_KD = 0.0030;
    /** Small breakaway assist for the rebuilt belt drive. */
    public static double STATIC_FRICTION_POWER = 0.025;
    private static final double POSITION_TOLERANCE_TICKS = 2.0;
    public static final double MOTOR_VELOCITY_FILTER_ALPHA = 0.50;
    /**
     * If the turret cannot move near the target, stop driving it instead of
     * heating the motor indefinitely. A moved setpoint automatically retries.
     */
    private static final double NEAR_TARGET_STALL_WINDOW_TICKS = 8.0;
    private static final double NEAR_TARGET_STALL_MAX_VELOCITY_TPS = 3.0;
    private static final double NEAR_TARGET_STALL_TIMEOUT_MS = 500.0;
    private static final double NEAR_TARGET_STALL_RETRY_DELTA_TICKS = 3.0;

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
    public static final boolean LATENCY_COMPENSATION_ENABLED = true;

    /**
     * Store the observed hive as one field-space point. Vision corrects that
     * point; odometry converts it back to a live robot-relative bearing every
     * loop, including while the tag is hidden or the robot crosses the field.
     */
    public static boolean ODOMETRY_FUSION_ENABLED = true;
    /** One fixed filter for the saved field target. Live vision itself remains authoritative. */
    private static final double FIELD_TARGET_FILTER_ALPHA = 0.35;
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
    private static final double ODO_MAX_HEADING_DEG_PER_S = 720.0;
    private static final double ODO_HEADING_JUMP_SLACK_DEG = 5.0;
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
    public static final double ODOMETRY_FORWARD_SIGN = 1.0;
    public static final double ODOMETRY_LEFT_SIGN = 1.0;
    public static final double ODOMETRY_HEADING_SIGN = 1.0;
    public static final double ODOMETRY_BEARING_SIGN = -1.0;
    // ========================= NORMAL CONSTANTS =========================

    private static final long MAX_RESULT_AGE_MS = 120;
    private static final long HOLD_AFTER_TARGET_LOSS_MS = 350;
    /** Rescan covers every encoder angle allowed by the cable-safe soft limits. */
    private static final double RESCAN_ENDPOINT_TOLERANCE_TICKS = 8.0;
    private static final double RESCAN_MAX_TIME_MS = 12000.0;
    private static final double MIN_DISTANCE_ANGLE_DEG = 2.0;
    /** Includes the roughly 204-inch diagonal of the 12-foot field, plus measurement margin. */
    private static final double MAX_DISTANCE_IN = 230.0;
    private static final double DISTANCE_FILTER_ALPHA = 0.25;
    /** Hold the last trusted range while a ball/transfer can shake or block the camera. */
    private static final double DISTANCE_SHOT_FREEZE_MS = 300.0;
    /**
     * Ty range is responsive enough for RPM but too noisy to recompute horizontal
     * camera parallax directly every frame. Filter only the depth used for aiming.
     */
    private static final double AIM_DEPTH_FILTER_ALPHA_STILL = 0.12;
    private static final double AIM_DEPTH_FILTER_ALPHA_MOVING = 0.65;
    private static final double AIM_DEPTH_RESEED_DELTA_IN = 36.0;
    /** A stationary robot cannot physically change range this far between vision frames. */
    private static final double STATIONARY_DISTANCE_OUTLIER_IN = 12.0;
    /** Even at full drive speed, a larger frame-to-frame jump is a bad edge-tag reading. */
    private static final double MOVING_DISTANCE_OUTLIER_IN = 40.0;
    private static final double MAX_TARGET_RATE_DEG_PER_SEC = 60.0;
    private static final double EXTRA_LATENCY_MS = 10.0;
    private static final double ENCODER_HISTORY_MS = 400.0;
    /**
     * goBILDA RGB Indicator Light solid colors with Hardware's 600-2400 us range.
     * Every value is deliberately inside the approximately 1100-1900 us color
     * band rather than on an OFF boundary.
     *
     * Blue   = manual aim
     * Red    = auto aim is not settled
     * Yellow = settled using saved odometry target; no live Limelight confidence
     * Green  = settled with a stable live Limelight target
     */
    public static final double LED_MANUAL_POSITION = 0.611;       // ~1700 us: blue
    private static final double LED_ALIGNING_POSITION = 0.300;    // ~1140 us: red
    private static final double LED_ODOMETRY_POSITION = 0.389;    // ~1300 us: yellow
    private static final double LED_VISION_LOCK_POSITION = 0.500; // 1500 us: green
    private static final double LED_MIN_POSITION_CHANGE = 0.004;

    private final Hardware robot;
    private final DcMotorEx turretMotor;
    private final TurretPositionController positionController = new TurretPositionController();
    private final ElapsedTime clock = new ElapsedTime();
    private final Deque<double[]> encoderHistory = new ArrayDeque<>();
    private final double[] txHistory = new double[TX_MEDIAN_WINDOW];
    private final double[] distanceHistory = new double[5];

    private boolean autoAimEnabled;
    private boolean movementPaused;
    private double pauseHoldTicks = Double.NaN;
    private double manualPower;
    /** Test-only trim owned by this subsystem instance; a new OpMode starts at zero. */
    private double runtimeAimTrimDeg;
    private int selectedTagId = -1;
    private Alliance alliance = Alliance.BLUE;
    private int selectedClusterFirstId = -1;
    private double selectedClusterRollDeg = Double.NaN;
    private double cameraSideOfClusterIn = Double.NaN;
    private double shooterSideOfClusterIn = Double.NaN;
    private double bankAimOffsetIn;
    /** -1 aims left of the hive, +1 aims right, 0 means not selected yet. */
    private int bankAimTargetSideSign;
    private double lastBankAimSideSwitchMs = Double.NEGATIVE_INFINITY;
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
    private double rawDistanceIn = Double.NaN;
    private boolean distanceInitialized;
    private double lastTransferFeedMs = -1;
    private boolean distanceFrozenAfterShot;

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
    private TrackingState trackingState = TrackingState.MANUAL;
    private TrackingState lastTrackingStateForEvents = TrackingState.MANUAL;
    private String trackingStateReason = "manual control";

    private boolean targetVisible;
    private boolean aimReady;
    private int stableAimFrames;
    private double lastVisionErrorDeg;
    private double lastLatencyMs;

    private double targetTicks;
    /** Latest complete vision solution, kept separate from the active command source. */
    private double visionTargetTicks = Double.NaN;
    /** Reachable encoder representation of targetTicks, after wrapping/clamping. */
    private double controlTargetTicks;
    private double targetVelocityTicksPerSecond;
    private boolean targetTicksInitialized;
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
    private double lastGoodX = Double.NaN, lastGoodY = Double.NaN;
    private double lastGoodHeadingRad = Double.NaN, lastGoodReadMs;
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
    /** Stop selected while the requested bearing crosses the turret's unreachable sector. */
    private double blindSectorHoldTicks = Double.NaN;
    private boolean rescanActive;
    private int rescanPhase;
    private double rescanTargetTicks = Double.NaN;
    private double rescanSecondTargetTicks = Double.NaN;
    private double rescanStartedMs = -1;
    private boolean fieldHandoffActive;
    private double fieldHandoffTargetTicks = Double.NaN;
    /** Selected soft-stop exit while the target is inside the unreachable rear sector. */
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
    private double dbgP, dbgD, dbgStiction, dbgScale, dbgRawPower;
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
    private boolean lastAtLimitForEvents;
    private boolean lastClampedForEvents;
    private boolean lastWrappedForEvents;
    private boolean hardLimitFault;
    private boolean lastHardLimitFaultForEvents;
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
        if (!enabled) {
            rescanActive = false;
            fieldHandoffActive = false;
        }
        autoAimOnMs = enabled ? clock.milliseconds() : -1;
        readyAfterEnableMs = Double.NaN;
        readyAfterStopMs = Double.NaN;
        readyReported = false;
        event("AUTOAIM " + (enabled ? "ON" : "OFF") + " ticks=" + lastTicks
                + " visible=" + targetVisible + " robotStopped=" + robotStopped);
        targetTicksInitialized = false;
        blindSectorHoldTicks = Double.NaN;
        resetNearTargetStallSafety();
        stableAimFrames = 0;
        aimReady = false;
        resetMeasurementHistory(true);
    }

    public void setManualPower(double power) {
        manualPower = clamp(power, -MAX_TURRET_POWER, MAX_TURRET_POWER);
    }

    /**
     * Stops turret output while tracking continues in the background. Holding
     * this gives the camera a stationary view; release resumes the best source.
     */
    public void setMovementPaused(boolean paused) {
        if (movementPaused == paused) return;
        movementPaused = paused;
        if (!paused) pauseHoldTicks = Double.NaN;
        event("TURRET_PAUSE " + (paused ? "ON" : "OFF"));
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
        rescanActive = false;
        fieldHandoffActive = false;
        blindSectorHoldTicks = Double.NaN;
        resetNearTargetStallSafety();
        clearFusionPending();
        clearPendingCluster();
        resetMeasurementHistory(false);
    }

    /**
     * Forgets a bad saved hive and sweeps the complete cable-safe turret range.
     * The nearer soft stop is visited first, then the opposite stop. A valid
     * scorable AprilTag cluster cancels the scan immediately and vision takes over.
     */
    public void beginFullRangeRescan() {
        setAutoAimEnabled(true);
        fieldHandoffActive = false;

        selectedClusterFirstId = -1;
        selectedClusterRollDeg = Double.NaN;
        selectedTagId = -1;
        targetVisible = false;
        lastTargetSeenMs = -1;
        resetSideAim();
        clearFusionPending();
        clearPendingCluster();
        resetMeasurementHistory(false);

        double currentTicks = robot.getTurretPositionTicks();
        boolean minIsNearer = Math.abs(currentTicks - MIN_TURRET_TICKS)
                <= Math.abs(MAX_TURRET_TICKS - currentTicks);
        rescanTargetTicks = minIsNearer ? MIN_TURRET_TICKS : MAX_TURRET_TICKS;
        rescanSecondTargetTicks = minIsNearer ? MAX_TURRET_TICKS : MIN_TURRET_TICKS;
        rescanPhase = 0;
        rescanStartedMs = clock.milliseconds();
        rescanActive = true;
        blindSectorHoldTicks = Double.NaN;
        targetTicks = rescanTargetTicks;
        controlTargetTicks = rescanTargetTicks;
        targetVelocityTicksPerSecond = 0;
        targetTicksInitialized = true;
        stableAimFrames = 0;
        aimReady = false;
        resetNearTargetStallSafety();
        event(String.format(java.util.Locale.US,
                "RESCAN_START ticks=%+.1f first=%+.1f second=%+.1f",
                currentTicks, rescanTargetTicks, rescanSecondTargetTicks));
    }

    /**
     * Called after odometry confirms that TeleOp crossed to the opposite half
     * of the field. Discard the previous CELL and turn to the opposite bearing;
     * vision takes over immediately when the new alliance CELL becomes visible.
     */
    public void beginOppositeFieldHandoff() {
        setAutoAimEnabled(true);

        selectedClusterFirstId = -1;
        selectedClusterRollDeg = Double.NaN;
        selectedTagId = -1;
        targetVisible = false;
        lastTargetSeenMs = -1;
        resetSideAim();
        clearFusionPending();
        clearPendingCluster();
        resetMeasurementHistory(false);

        double currentTicks = robot.getTurretPositionTicks();
        double requestedTicks = currentTicks
                + AIM_DIRECTION_SIGN * 180.0 * TICKS_PER_DEGREE;
        blindSectorHoldTicks = Double.NaN;
        fieldHandoffTargetTicks = resolveTargetTicksWithinLimits(
                requestedTicks, currentTicks);
        fieldHandoffActive = true;
        rescanActive = false;
        targetTicks = fieldHandoffTargetTicks;
        controlTargetTicks = fieldHandoffTargetTicks;
        targetVelocityTicksPerSecond = 0;
        targetTicksInitialized = true;
        resetNearTargetStallSafety();
        stableAimFrames = 0;
        aimReady = false;
        event(String.format(java.util.Locale.US,
                "FIELD_HANDOFF_180 ticks=%+.1f->%+.1f",
                currentTicks, fieldHandoffTargetTicks));
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
        // The saved hive is a field-space point, independent of encoder zero.
        // Reprocess vision immediately so no target from the old tick frame survives.
        hardLimitFault = false;
        targetTicks = 0;
        controlTargetTicks = 0;
        targetVelocityTicksPerSecond = 0;
        targetTicksInitialized = false;
        blindSectorHoldTicks = Double.NaN;
        resetNearTargetStallSafety();
        appliedPower = 0;
        manualPower = 0;
        encoderHistory.clear();
        motorVelocityInitialized = false;
        rescanActive = false;
        fieldHandoffActive = false;
        resetMeasurementHistory(true);
    }

    /** Call once per OpMode loop. */
    public void update(double dtSeconds) {
        long updateStartNs = System.nanoTime();
        loopCount++;
        dbgDtMs = dtSeconds * 1000.0;
        double dt = clamp(dtSeconds, 0.001, 0.100);
        double nowMs = clock.milliseconds();
        if (robot.transfer.getPower() > 0.05) lastTransferFeedMs = nowMs;
        distanceFrozenAfterShot = lastTransferFeedMs >= 0
                && nowMs - lastTransferFeedMs < DISTANCE_SHOT_FREEZE_MS;
        int currentTicks = robot.getTurretPositionTicks();
        lastTicks = currentTicks;
        if (movementPaused && !Double.isFinite(pauseHoldTicks)) {
            pauseHoldTicks = currentTicks;
        }
        hardLimitFault = currentTicks < MIN_TURRET_TICKS - HARD_LIMIT_OVERRUN_TICKS
                || currentTicks > MAX_TURRET_TICKS + HARD_LIMIT_OVERRUN_TICKS;
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
        updateRescan(nowMs, currentTicks);
        updateTrackingState(nowMs);
        long t2 = System.nanoTime();

        if (trackingState == TrackingState.PAUSED) {
            applyPausedControl(currentTicks);
        } else if (autoAimEnabled) {
            applyAutomaticControl(nowMs, currentTicks);
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

    /**
     * Selects exactly one command source. Vision always wins while visible.
     * A very short dropout holds the last vision setpoint so a single missed
     * camera frame cannot make the command chatter between vision and odometry.
     * A real loss then falls back to the saved field target through odometry.
     */
    private void updateTrackingState(double nowMs) {
        if (hardLimitFault) {
            trackingState = TrackingState.FAULT;
            trackingStateReason = "encoder outside hard limit";
        } else if (movementPaused) {
            trackingState = TrackingState.PAUSED;
            trackingStateReason = "driver holding BACK";
        } else if (!autoAimEnabled) {
            trackingState = TrackingState.MANUAL;
            trackingStateReason = "driver control";
        } else if (targetVisible) {
            fieldHandoffActive = false;
            trackingState = TrackingState.VISION;
            trackingStateReason = "fresh allowed AprilTag";
        } else if (rescanActive) {
            trackingState = TrackingState.RESCAN;
            trackingStateReason = rescanPhase == 0
                    ? "rescan: first safe endpoint"
                    : "rescan: opposite safe endpoint";
        } else if (fieldHandoffActive) {
            trackingState = TrackingState.FIELD_HANDOFF;
            trackingStateReason = "odometry crossed field center: turning 180";
        } else if (lastTargetSeenMs >= 0
                && nowMs - lastTargetSeenMs <= MAX_RESULT_AGE_MS) {
            trackingState = TrackingState.HOLD;
            trackingStateReason = "brief vision dropout";
        } else if (ODOMETRY_FUSION_ENABLED
                && fusValid
                && odometryPoseValid && Double.isFinite(fusTargetTicks)) {
            trackingState = TrackingState.ODOMETRY;
            trackingStateReason = "saved hive projected by odometry";
        } else {
            trackingState = TrackingState.HOLD;
            if (!fusValid) {
                trackingStateReason = "no saved hive";
            } else if (!odometryPoseValid) {
                trackingStateReason = "odometry unavailable";
            } else {
                trackingStateReason = "fallback disabled or invalid";
            }
        }
        odometryTrackingActive = trackingState == TrackingState.ODOMETRY;
    }

    private void updateRescan(double nowMs, int currentTicks) {
        if (!rescanActive) return;
        if (targetVisible) {
            rescanActive = false;
            event("RESCAN_ACQUIRED tags=" + visibleTagIds);
            return;
        }
        if (nowMs - rescanStartedMs >= RESCAN_MAX_TIME_MS) {
            rescanActive = false;
            event("RESCAN_TIMEOUT no valid tag");
            return;
        }

        boolean endpointReached = Math.abs(currentTicks - rescanTargetTicks)
                <= RESCAN_ENDPOINT_TOLERANCE_TICKS
                && Math.abs(filteredMotorVelocityTicksPerSecond)
                <= READY_MAX_VELOCITY_TPS;
        if (!endpointReached) return;

        if (rescanPhase == 0) {
            rescanPhase = 1;
            rescanTargetTicks = rescanSecondTargetTicks;
            resetNearTargetStallSafety();
            event(String.format(java.util.Locale.US,
                    "RESCAN_REVERSE target=%+.1f", rescanTargetTicks));
        } else {
            rescanActive = false;
            event("RESCAN_COMPLETE no valid tag");
        }
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
        boolean targetKnown = trackingState == TrackingState.VISION
                || trackingState == TrackingState.ODOMETRY;
        boolean sourceStable = trackingState == TrackingState.ODOMETRY
                || (trackingState == TrackingState.VISION
                && stableAimFrames >= AIM_STABLE_FRAMES);
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
        if (trackingState != lastTrackingStateForEvents) {
            event("TRACKING_STATE " + lastTrackingStateForEvents + " -> "
                    + trackingState + " (" + trackingStateReason + ")");
            lastTrackingStateForEvents = trackingState;
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
        if (hardLimitFault != lastHardLimitFaultForEvents) {
            lastHardLimitFaultForEvents = hardLimitFault;
            event(hardLimitFault
                    ? "HARD_LIMIT_FAULT ticks=" + currentTicks + " AUTO_BLOCKED_MANUAL_INWARD_ONLY"
                    : "HARD_LIMIT_FAULT_CLEARED");
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

    /** Snapshot of intentional calibration/controller values, excluding internal guards. */
    public static java.util.Map<String, String> configSnapshot() {
        java.util.Map<String, String> m = new java.util.LinkedHashMap<>();
        m.put("turret.TUNE_VERSION", TUNE_VERSION);
        m.put("turret.TICKS_PER_DEGREE", String.valueOf(TICKS_PER_DEGREE));
        m.put("turret.MIN_TURRET_TICKS", String.valueOf(MIN_TURRET_TICKS));
        m.put("turret.MAX_TURRET_TICKS", String.valueOf(MAX_TURRET_TICKS));
        m.put("turret.AIM_ANGLE_SCALE", String.valueOf(AIM_ANGLE_SCALE));
        m.put("turret.AIM_OFFSET_DEG", String.valueOf(AIM_OFFSET_DEG));
        m.put("turret.CAMERA_HEIGHT_IN", String.valueOf(CAMERA_HEIGHT_IN));
        m.put("turret.CAMERA_UP_ANGLE_DEG", String.valueOf(CAMERA_UP_ANGLE_DEG));
        m.put("turret.CAMERA_LEFT_OF_SHOOTER_IN", String.valueOf(CAMERA_LEFT_OF_SHOOTER_IN));
        m.put("turret.DEFAULT_TAG_HEIGHT_IN", String.valueOf(DEFAULT_TAG_HEIGHT_IN));
        m.put("turret.BANK_AIM_CENTER_OFFSET_IN", String.valueOf(BANK_AIM_CENTER_OFFSET_IN));
        m.put("turret.BANK_AIM_MAX_OFFSET_IN", String.valueOf(BANK_AIM_MAX_OFFSET_IN));
        m.put("turret.BANK_AIM_FULL_SIDE_IN", String.valueOf(BANK_AIM_FULL_SIDE_IN));
        m.put("turret.BANK_AIM_SWITCH_HYSTERESIS_IN",
                String.valueOf(BANK_AIM_SWITCH_HYSTERESIS_IN));
        m.put("turret.BANK_AIM_SWITCH_HOLD_MS", String.valueOf(BANK_AIM_SWITCH_HOLD_MS));
        m.put("turret.POSITION_KP", String.valueOf(POSITION_KP));
        m.put("turret.VELOCITY_KD", String.valueOf(VELOCITY_KD));
        m.put("turret.STATIC_FRICTION_POWER", String.valueOf(STATIC_FRICTION_POWER));
        m.put("turret.RESCAN_MAX_POWER", String.valueOf(RESCAN_MAX_POWER));
        m.put("turret.ODOMETRY_FUSION_ENABLED", String.valueOf(ODOMETRY_FUSION_ENABLED));
        m.put("turret.ODOMETRY_FORWARD_SIGN", String.valueOf(ODOMETRY_FORWARD_SIGN));
        m.put("turret.ODOMETRY_LEFT_SIGN", String.valueOf(ODOMETRY_LEFT_SIGN));
        m.put("turret.ODOMETRY_HEADING_SIGN", String.valueOf(ODOMETRY_HEADING_SIGN));
        m.put("turret.ODOMETRY_BEARING_SIGN", String.valueOf(ODOMETRY_BEARING_SIGN));
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
        r.put("tur_paused", movementPaused);
        r.put("tur_rescan", rescanActive);
        r.put("tur_rescan_phase", rescanPhase);
        r.put("tur_field_handoff", fieldHandoffActive);
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
        // Retained as zero-valued columns so older log-analysis sheets still import.
        r.put("tur_kv", 0);
        r.put("tur_i", 0);
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
        r.put("aim_base_offset_deg", AIM_OFFSET_DEG);
        r.put("aim_runtime_trim_deg", runtimeAimTrimDeg);
        r.put("aim_total_offset_deg", getTotalAimOffsetDeg());
        r.put("aim_camera_side_in", cameraSideOfClusterIn);
        r.put("aim_shooter_side_in", shooterSideOfClusterIn);
        r.put("aim_bank_offset_in", bankAimOffsetIn);
        r.put("aim_bank_correction_deg", bankAimCorrectionDeg);
        r.put("aim_bank_side_sign", bankAimTargetSideSign);
        r.put("aim_bank_switch_remaining_ms", getBankAimSwitchRemainingMs());
        r.put("vis_stable_frames", stableAimFrames);
        r.put("vis_geometry_ok", clusterGeometryValid);
        r.put("vis_dist_raw_in", rawDistanceIn);
        r.put("vis_dist_in", filteredDistanceIn);
        r.put("vis_dist_frozen_for_shot", distanceFrozenAfterShot);
        r.put("vis_fused_range_disagreement_in", getRangeDisagreementIn());
        r.put("vis_target_age_ms", Math.min(getTargetAgeMs(), 99999));

        r.put("aim_ready", aimReady);
        r.put("aim_source", trackingState.ordinal());
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
        r.put("tur_hard_limit_fault", hardLimitFault);
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
        // Only save a field point using the range that passed updateDistance's
        // bounds, median, jump rejection, and shot-freeze checks. Using the raw
        // cluster range here could anchor odometry far beyond the real hive.
        double fusionDistanceIn = distanceInitialized ? filteredDistanceIn : Double.NaN;
        bankAimCorrectionDeg = bankAimCorrectionForDistance(fusionDistanceIn);
        double calibratedBearingAtCaptureDeg = calibratedAimBearingDeg(bearingAtCaptureDeg)
                + bankAimCorrectionDeg;
        dbgRawBearingDeg = bearingAtCaptureDeg;
        dbgCalibratedBearingDeg = calibratedBearingAtCaptureDeg;
        lastRobotBearingDeg = bearingAtCaptureDeg;
        // Fusion must remember the physical hive center, not whichever edge is
        // currently selected for the shot. The edge correction is added later.
        // A feeding ball can briefly shake the camera or cover part of a tag.
        // Keep predicting from the last trusted field point during that window.
        if (!distanceFrozenAfterShot) {
            updateFusionFromFrame(nowMs, captureTimeMs, bearingAtCaptureDeg, fusionDistanceIn);
        }

        // Convert the raw target bearing into the calibrated shooter bearing. Expressing
        // the error against the CURRENT turret position also performs latency compensation:
        // at scale 1.0 this is exactly horizontalError + turretMovement.
        double physicalTurretBearingNowDeg = currentTicks
                / (AIM_DIRECTION_SIGN * TICKS_PER_DEGREE);
        lastVisionErrorDeg = calibratedBearingAtCaptureDeg
                - physicalTurretBearingNowDeg;
        stableAimFrames = Math.abs(lastVisionErrorDeg) <= AIM_TOLERANCE_DEG
                ? stableAimFrames + 1 : 0;

        double measuredTargetTicks = currentTicks
                + AIM_DIRECTION_SIGN * lastVisionErrorDeg * TICKS_PER_DEGREE;
        dbgMeasuredTargetTicks = measuredTargetTicks;
        fusMinusMeasTicks = Double.isNaN(fusTargetTicks)
                ? Double.NaN : fusTargetTicks - measuredTargetTicks;
        // The three-frame tx median above is the only vision setpoint filter.
        // Live vision is authoritative; the motor's PD controller handles the
        // physical smoothing instead of another stack of gains/deadbands here.
        visionTargetTicks = measuredTargetTicks;
        dbgGain = 1.0;
    }

    private void updateDistance(
            double targetYDegrees, int tagId, double clusterDistanceIn,
            double targetAgeBeforeFrameMs) {
        double measuredDistanceIn = clusterDistanceIn;
        if (!Double.isFinite(measuredDistanceIn)) {
            measuredDistanceIn = estimateTagFloorForwardFromTy(targetYDegrees, tagId);
        }
        rawDistanceIn = measuredDistanceIn;
        if (measuredDistanceIn <= 0 || measuredDistanceIn >= MAX_DISTANCE_IN) return;

        // Do not allow shot vibration or a ball crossing the image to change RPM.
        // The existing median history is preserved and resumes after the short hold.
        if (distanceFrozenAfterShot) return;

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
                && Math.abs(measuredDistanceIn - filteredDistanceIn) > distanceOutlierLimitIn) {
            return;
        }

        double medianDistance = addAndMedian(
                distanceHistory, distanceHistoryCount, measuredDistanceIn);
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
        double rawHeadingRad = robot.odo.getHeading(AngleUnit.RADIANS);
        if (!Double.isNaN(lastGoodX) && Double.isFinite(rawX) && Double.isFinite(rawY)) {
            double gapS = Math.max((readStartMs - lastGoodReadMs) / 1000.0, 0.001);
            double allowed = ODO_JUMP_SLACK_IN + ODO_MAX_SPEED_IN_PER_S * gapS;
            double jump = Math.hypot(rawX - lastGoodX, rawY - lastGoodY);
            double headingJumpDeg = Double.isFinite(rawHeadingRad)
                    && Double.isFinite(lastGoodHeadingRad)
                    ? Math.abs(Math.toDegrees(Math.atan2(
                            Math.sin(rawHeadingRad - lastGoodHeadingRad),
                            Math.cos(rawHeadingRad - lastGoodHeadingRad)))) : 0;
            double allowedHeadingDeg = ODO_HEADING_JUMP_SLACK_DEG
                    + ODO_MAX_HEADING_DEG_PER_S * gapS;
            if ((jump > allowed || headingJumpDeg > allowedHeadingDeg)
                    && odoConsecutiveJumps < ODO_RESYNC_AFTER_JUMPS) {
                odoConsecutiveJumps++;
                odoJumpRejects++;
                if (odoJumpRejects <= 25) {
                    event(String.format(java.util.Locale.US,
                            "ODO_JUMP_REJECTED pos=%.1f/%.1fin heading=%.1f/%.1fdeg gap=%.0fms",
                            jump, allowed, headingJumpDeg, allowedHeadingDeg, gapS * 1000));
                }
                return;
            }
        }
        if (odoConsecutiveJumps >= ODO_RESYNC_AFTER_JUMPS) {
            // A forced pose resync changes the coordinate frame underneath the
            // saved field point, so discard it instead of steering to stale coordinates.
            fusValid = false;
            event("ODO_RESYNC_FIELD_TARGET_CLEARED");
        }
        odoConsecutiveJumps = 0;
        lastGoodX = rawX;
        lastGoodY = rawY;
        lastGoodHeadingRad = rawHeadingRad;
        lastGoodReadMs = readStartMs;
        lastGoodOdoMs = readStartMs;

        double forwardVelocity = robot.odo.getVelX(DistanceUnit.INCH);
        double leftVelocity = robot.odo.getVelY(DistanceUnit.INCH);
        double headingRateDegPerSec = robot.odo.getHeadingVelocity(
                UnnormalizedAngleUnit.DEGREES);
        double fieldX = robot.odo.getPosX(DistanceUnit.INCH);
        double fieldY = robot.odo.getPosY(DistanceUnit.INCH);
        double headingRad = rawHeadingRad;

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
    private double calibratedAimBearingDeg(double rawBearingDeg) {
        double minBearingDeg = MIN_TURRET_TICKS
                / (AIM_DIRECTION_SIGN * TICKS_PER_DEGREE);
        double maxBearingDeg = MAX_TURRET_TICKS
                / (AIM_DIRECTION_SIGN * TICKS_PER_DEGREE);
        double reachableCenterDeg = (minBearingDeg + maxBearingDeg) / 2.0;
        double canonicalBearingDeg = rawBearingDeg + 360.0
                * Math.rint((reachableCenterDeg - rawBearingDeg) / 360.0);
        return AIM_ANGLE_SCALE * canonicalBearingDeg + getTotalAimOffsetDeg();
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
                fusX += FIELD_TARGET_FILTER_ALPHA * (meas[0] - fusX);
                fusY += FIELD_TARGET_FILTER_ALPHA * (meas[1] - fusY);
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
        if (!Double.isFinite(fusDistanceIn)
                || fusDistanceIn < 1.0 || fusDistanceIn > MAX_DISTANCE_IN) {
            // Preserve the field anchor in case odometry becomes sane again.
            fusBearingDeg = fusDistanceIn = fusTargetTicks = Double.NaN;
            fusPrevBearingMs = -1;
            fusRateDps = 0;
            return;
        }
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

    private void applyPausedControl(int currentTicks) {
        if (!Double.isFinite(pauseHoldTicks)) pauseHoldTicks = currentTicks;
        targetTicks = pauseHoldTicks;
        controlTargetTicks = pauseHoldTicks;
        targetVelocityTicksPerSecond = 0;
        targetTicksInitialized = true;
        targetClamped = false;
        targetWrapped = false;
        dbgRequestedTargetTicks = pauseHoldTicks;
        dbgP = dbgD = dbgStiction = dbgRawPower = dbgDesiredPower = 0;
        appliedPower = 0;
        resetNearTargetStallSafety();
        writePower(0);
    }

    private void applyAutomaticControl(double nowMs, int currentTicks) {
        if (!targetTicksInitialized) targetTicks = currentTicks;

        if (trackingState == TrackingState.FAULT) {
            targetTicks = currentTicks;
            targetTicksInitialized = true;
            targetVelocityTicksPerSecond = 0;
            controlTargetTicks = currentTicks;
            appliedPower = 0;
            writePower(0);
            return;
        }

        if (trackingState == TrackingState.VISION && Double.isFinite(visionTargetTicks)) {
            targetTicks = visionTargetTicks;
            targetTicksInitialized = true;
            targetVelocityTicksPerSecond = 0;
        } else if (trackingState == TrackingState.RESCAN) {
            targetTicks = rescanTargetTicks;
            targetTicksInitialized = true;
            targetVelocityTicksPerSecond = 0;
        } else if (trackingState == TrackingState.FIELD_HANDOFF) {
            targetTicks = fieldHandoffTargetTicks;
            targetTicksInitialized = true;
            targetVelocityTicksPerSecond = 0;
        } else if (trackingState == TrackingState.ODOMETRY) {
            targetTicks = fusTargetTicks;
            targetTicksInitialized = true;
            targetVelocityTicksPerSecond = robotStopped
                    ? 0 : AIM_DIRECTION_SIGN * fusRateDps * TICKS_PER_DEGREE;
        } else {
            // HOLD retains the last safe setpoint. It does not invent or blend
            // another target source.
            targetVelocityTicksPerSecond = 0;
        }

        dbgRequestedTargetTicks = targetTicks;
        controlTargetTicks = resolveTargetTicksWithinLimits(
                dbgRequestedTargetTicks, currentTicks);
        targetWrapped = isEquivalentRevolution(
                dbgRequestedTargetTicks, controlTargetTicks)
                && Math.abs(controlTargetTicks - dbgRequestedTargetTicks) > 1e-6;
        targetClamped = !targetWrapped
                && Math.abs(controlTargetTicks - dbgRequestedTargetTicks) > 1e-6;
        if (targetClamped) targetVelocityTicksPerSecond = 0;
        double positionError = controlTargetTicks - currentTicks;

        dbgP = dbgD = dbgStiction = dbgRawPower = dbgDesiredPower = 0;
        dbgScale = 1;
        dbgSaturated = false;
        boolean stallSafetyHolding = updateNearTargetStallSafety(positionError, nowMs);
        if (stallSafetyHolding) {
            appliedPower = 0;
        } else {
            TurretPositionController.Output output = positionController.calculate(
                    positionError,
                    filteredMotorVelocityTicksPerSecond,
                    POSITION_KP,
                    VELOCITY_KD,
                    STATIC_FRICTION_POWER,
                    POSITION_TOLERANCE_TICKS);
            dbgP = output.proportional;
            dbgD = output.damping;
            dbgStiction = output.staticFriction;
            dbgRawPower = output.power;
            appliedPower = MOTOR_DIRECTION_SIGN * output.power;
        }
        dbgDesiredPower = appliedPower;
        double activePowerLimit = trackingState == TrackingState.RESCAN
                ? clamp(Math.abs(RESCAN_MAX_POWER), 0.05, MAX_TURRET_POWER)
                : MAX_TURRET_POWER;
        dbgSaturated = Math.abs(appliedPower) > activePowerLimit;
        appliedPower = limitAtSoftStops(
                clamp(appliedPower, -activePowerLimit, activePowerLimit), currentTicks);
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
        blindSectorHoldTicks = Double.NaN;
        resetNearTargetStallSafety();
        dbgP = dbgD = dbgStiction = dbgRawPower = 0;
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
        } else if (aimReady && trackingState == TrackingState.VISION) {
            // Green is reserved for a stable, currently visible Limelight target.
            position = LED_VISION_LOCK_POSITION;
        } else if (aimReady && trackingState == TrackingState.ODOMETRY) {
            // The turret is settled, but the target is only being carried by odometry.
            position = LED_ODOMETRY_POSITION;
        } else {
            position = LED_ALIGNING_POSITION;
        }
        writeIndicatorPosition(position);
    }

    private void writeIndicatorPosition(double position) {
        boolean pwmEnabled = robot.shooterLed.isPwmEnabled();
        if (pwmEnabled
                && Double.isFinite(indicatorPosition)
                && Math.abs(position - indicatorPosition) < LED_MIN_POSITION_CHANGE) {
            return;
        }
        indicatorPosition = position;
        if (!pwmEnabled) robot.shooterLed.setPwmEnable();
        robot.shooterLed.setPosition(position);
    }

    private double limitAtSoftStops(double power, int currentTicks) {
        double tickDirection = MOTOR_DIRECTION_SIGN * power;
        // Beyond a hard limit, block outward power but still allow a slow manual
        // recovery toward the legal range. Automatic control remains disabled.
        if (currentTicks < MIN_TURRET_TICKS - HARD_LIMIT_OVERRUN_TICKS) {
            return tickDirection > 0 ? power : 0;
        }
        if (currentTicks > MAX_TURRET_TICKS + HARD_LIMIT_OVERRUN_TICKS) {
            return tickDirection < 0 ? power : 0;
        }
        if (currentTicks >= MAX_TURRET_TICKS && tickDirection > 0) return 0;
        if (currentTicks <= MIN_TURRET_TICKS && tickDirection < 0) return 0;
        return power;
    }

    /**
     * Maps an angular target to the closest equivalent encoder target inside
     * the physical soft stops. If no equivalent revolution is reachable, clamp
     * to the nearest numeric stop. This is intentionally simple: a target just
     * past -100 must become -100, never the opposite +400 stop.
     */
    private double resolveTargetTicksWithinLimits(
            double requestedTicks, double currentTicks) {
        if (!Double.isFinite(requestedTicks)) {
            blindSectorHoldTicks = Double.NaN;
            return clampStatic(currentTicks, MIN_TURRET_TICKS, MAX_TURRET_TICKS);
        }
        if (requestedTicks >= MIN_TURRET_TICKS
                && requestedTicks <= MAX_TURRET_TICKS) {
            blindSectorHoldTicks = Double.NaN;
            return requestedTicks;
        }

        double revolution = Math.abs(TURRET_TICKS_PER_REVOLUTION);
        if (revolution < 1e-6) {
            blindSectorHoldTicks = Double.NaN;
            return clampStatic(requestedTicks, MIN_TURRET_TICKS, MAX_TURRET_TICKS);
        }

        long firstTurn = (long) Math.ceil(
                (MIN_TURRET_TICKS - requestedTicks) / revolution);
        long lastTurn = (long) Math.floor(
                (MAX_TURRET_TICKS - requestedTicks) / revolution);
        if (firstTurn <= lastTurn) {
            blindSectorHoldTicks = Double.NaN;
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

        // No equivalent angle is physically reachable: the target is behind
        // the turret's cable-safe sector. Select the nearest numeric stop once
        // and hold it through the bearing's +/-360 representation change. This
        // prevents +MAX <-> MIN command flips while retaining full odometry
        // tracking as soon as the target re-enters the reachable sector.
        if (!Double.isFinite(blindSectorHoldTicks)) {
            blindSectorHoldTicks = clampStatic(
                    requestedTicks, MIN_TURRET_TICKS, MAX_TURRET_TICKS);
        }
        return blindSectorHoldTicks;
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
            double correctedTxDeg = tag.getTargetXDegrees();
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
                double correctedTxDeg = tag.getTargetXDegrees();
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
            cameraSideOfClusterIn += SIDE_AIM_FILTER_ALPHA
                    * (measuredSideIn - cameraSideOfClusterIn);
        }

        // The Limelight is left of the shooter, so convert the camera's
        // target-space position into the shooter's position before deciding
        // which side/edge should be used.
        shooterSideOfClusterIn = cameraSideOfClusterIn + CAMERA_LEFT_OF_SHOOTER_IN;

        // Pick the opposite side of the hive, then latch it. It may only switch
        // once the shooter clearly crosses the opposite side of center AND the
        // time guard expires. This rejects frame noise and small driver motion.
        double nowMs = clock.milliseconds();
        double switchBoundaryIn = Math.max(0, Math.abs(BANK_AIM_SWITCH_HYSTERESIS_IN));
        if (bankAimTargetSideSign == 0) {
            // An exact zero is the only ambiguous point; choose left once, then latch.
            bankAimTargetSideSign = shooterSideOfClusterIn < 0 ? 1 : -1;
            lastBankAimSideSwitchMs = nowMs;
        } else {
            boolean holdExpired = nowMs - lastBankAimSideSwitchMs
                    >= Math.max(0, BANK_AIM_SWITCH_HOLD_MS);
            boolean crossedForRightAim = bankAimTargetSideSign < 0
                    && shooterSideOfClusterIn < -switchBoundaryIn;
            boolean crossedForLeftAim = bankAimTargetSideSign > 0
                    && shooterSideOfClusterIn > switchBoundaryIn;
            if (holdExpired && (crossedForRightAim || crossedForLeftAim)) {
                bankAimTargetSideSign = -bankAimTargetSideSign;
                lastBankAimSideSwitchMs = nowMs;
                event("BANK_SIDE_SWITCH target=" + getBankAimSide()
                        + " shooter_side_in=" + shooterSideOfClusterIn);
            }
        }

        // Keep a non-zero side aim at center, then grow symmetrically toward
        // the maximum as the shooter moves away from the hive centerline.
        double maxOffsetIn = Math.max(0, Math.abs(BANK_AIM_MAX_OFFSET_IN));
        double centerOffsetIn = Math.min(
                Math.max(0, Math.abs(BANK_AIM_CENTER_OFFSET_IN)), maxOffsetIn);
        double sideFraction = clamp(
                Math.abs(shooterSideOfClusterIn)
                        / Math.max(0.1, Math.abs(BANK_AIM_FULL_SIDE_IN)),
                0, 1);
        double requestedMagnitudeIn = centerOffsetIn
                + (maxOffsetIn - centerOffsetIn) * sideFraction;
        bankAimOffsetIn = bankAimTargetSideSign * requestedMagnitudeIn;
    }

    private void resetSideAim() {
        cameraSideOfClusterIn = Double.NaN;
        shooterSideOfClusterIn = Double.NaN;
        bankAimOffsetIn = 0;
        bankAimTargetSideSign = 0;
        lastBankAimSideSwitchMs = Double.NEGATIVE_INFINITY;
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
        visionTargetTicks = Double.NaN;
        txHistoryCount = 0;
        distanceHistoryCount = 0;
        distanceInitialized = false;
        rawDistanceIn = Double.NaN;
        lastRobotBearingDeg = Double.NaN;
        if (!preserveFieldTarget) {
            fusValid = false;
        }
        odometryTrackingActive = false;
        lastClusterTagPlaneDepthIn = Double.NaN;
        clusterGeometryValid = false;
        aimReady = false;
        stableAimFrames = 0;
    }

    public boolean hasTarget() { return targetVisible; }
    public boolean isRescanActive() { return rescanActive; }
    public boolean isFieldHandoffActive() { return fieldHandoffActive; }
    public boolean hasDistance() { return distanceInitialized; }
    public boolean hasFieldTargetMemory() { return fusValid; }
    public boolean hasHardLimitFault() { return hardLimitFault; }
    public boolean isAimReady() { return aimReady; }
    public int getSelectedTagId() { return selectedTagId; }
    public Alliance getAlliance() { return alliance; }
    public int getSelectedClusterFirstId() { return selectedClusterFirstId; }
    public double getSelectedClusterRollDeg() { return selectedClusterRollDeg; }
    public double getCameraSideOfClusterIn() { return cameraSideOfClusterIn; }
    public double getShooterSideOfClusterIn() { return shooterSideOfClusterIn; }
    public double getBankAimOffsetIn() { return bankAimOffsetIn; }
    public double getBankAimCorrectionDeg() { return bankAimCorrectionDeg; }
    public String getBankAimSide() {
        if (bankAimTargetSideSign == 0) return "UNSELECTED";
        return bankAimTargetSideSign > 0 ? "RIGHT" : "LEFT";
    }
    public double getBankAimSwitchRemainingMs() {
        if (bankAimTargetSideSign == 0 || !Double.isFinite(lastBankAimSideSwitchMs)) return 0;
        return Math.max(0, Math.max(0, BANK_AIM_SWITCH_HOLD_MS)
                - (clock.milliseconds() - lastBankAimSideSwitchMs));
    }
    public double getIndicatorPosition() { return indicatorPosition; }
    public double getIndicatorPulseUs() {
        return Hardware.SHOOTER_LED_PWM_MIN_US
                + indicatorPosition * (Hardware.SHOOTER_LED_PWM_MAX_US
                - Hardware.SHOOTER_LED_PWM_MIN_US);
    }
    /** Applies a temporary test trim without changing the competition constant. */
    public void setRuntimeAimTrimDeg(double trimDeg) {
        runtimeAimTrimDeg = Double.isFinite(trimDeg)
                ? clamp(trimDeg, -15.0, 15.0) : 0.0;
    }
    public double getRuntimeAimTrimDeg() { return runtimeAimTrimDeg; }
    public double getTotalAimOffsetDeg() { return AIM_OFFSET_DEG + runtimeAimTrimDeg; }
    /** Re-seeds vision/fusion after a camera, tag, bank, or pivot geometry change. */
    public void refreshAimCalibration() {
        targetTicksInitialized = false;
        rescanActive = false;
        fieldHandoffActive = false;
        blindSectorHoldTicks = Double.NaN;
        resetNearTargetStallSafety();
        resetSideAim();
        clearFusionPending();
        resetMeasurementHistory(false);
    }
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
    public double getRawDistanceIn() { return rawDistanceIn; }
    public boolean isDistanceFrozenAfterShot() { return distanceFrozenAfterShot; }
    public double getRangeDisagreementIn() {
        return distanceInitialized && Double.isFinite(fusDistanceIn)
                ? filteredDistanceIn - fusDistanceIn : Double.NaN;
    }
    public boolean hasFusedDistance() {
        return ODOMETRY_FUSION_ENABLED && fusValid
                && odometryPoseValid
                && Double.isFinite(fusDistanceIn)
                && fusDistanceIn > 0 && fusDistanceIn <= MAX_DISTANCE_IN;
    }
    public double getFusedDistanceIn() { return fusDistanceIn; }
    /**
     * Stable range for the RPM table. Vision seeds/corrects the field target;
     * odometry updates its distance every loop and carries it through brief occlusion.
     */
    public double getShooterDistanceIn() {
        // The fused field target already rejects large unconfirmed vision jumps.
        // Prefer it whenever valid instead of selecting the disagreeing live range.
        if (hasFusedDistance()) return fusDistanceIn;
        return filteredDistanceIn;
    }
    public double getRobotForwardVelocityInPerSec() { return robotForwardVelocityInPerSec; }
    public double getRobotLeftVelocityInPerSec() { return robotLeftVelocityInPerSec; }
    public double getRobotFieldYIn() { return robotFieldYIn; }
    public double getRobotHeadingRateDegPerSec() {
        return Math.toDegrees(robotHeadingRateRadPerSec);
    }
    public boolean isOdometryRateValid() { return odometryRateValid; }
    public boolean isOdometryPoseValid() { return odometryPoseValid; }
    public boolean isOdometryTrackingActive() { return odometryTrackingActive; }
    public TrackingState getTrackingState() { return trackingState; }
    public String getTrackingStateReason() { return trackingStateReason; }
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
        rescanActive = false;
        fieldHandoffActive = false;
        blindSectorHoldTicks = Double.NaN;
        resetNearTargetStallSafety();
        writeIndicatorPosition(LED_MANUAL_POSITION);
        robot.limelight.stop();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
