package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;

/**
 * Pinpoint settings that Pedro's config has no field for (yaw scalar, start pose,
 * turret pivot, lean read). Pod offsets / directions / resolution live in Pedro's
 * Constants.localizerConfig, which {@link Hardware} reads, so autonomous and TeleOp
 * always share one calibration.
 *
 * Why this exists: the Pinpoint forgets nothing while powered but is NOT
 * configured by the SDK. Before this class, only Pedro's Follower configured
 * it, so any OpMode without a Follower ran on stale/undefined settings and an
 * undefined pose frame.
 *
 * Calibrate with the "ODO CALIBRATION" OpMode + tools/analyze_odometry_log.py.
 */
public final class OdometryConfig {
    private OdometryConfig() { }

    /**
     * Pod offsets, directions, pod type and linear resolution are NOT here: they live in
     * Pedro's Constants.localizerConfig (the single source of truth) and are read by
     * Hardware from there. See PedroPathing/Constants.java.
     */

    /** Factory resolution of the goBILDA 4-bar pod, ticks per mm (used when Pedro sets no override). */
    public static final double FACTORY_TICKS_PER_MM = 19.89436789;

    /**
     * YAW CALIBRATION. 1.0 = IMU as-is. After a multi-turn spin test the analyzer
     * prints the value to paste here (e.g. 0.9967 if the IMU over-reports).
     */
    public static final double YAW_SCALAR = 1.0;

    /**
     * Pose the robot is placed at before INIT (field inches / degrees, Pedro frame).
     * Must match where you actually place the robot. Matches the old TeleOp default.
     */
    public static final double START_X_IN = 56.0;
    public static final double START_Y_IN = 8.0;
    public static final double START_HEADING_DEG = 0.0;

    /**
     * Read only pose+velocity registers instead of the driver's default bulk
     * read (which also pulls quaternion/pitch/roll). Self-checked at INIT and
     * silently falls back if the lean read disagrees with the full read.
     */
    public static final boolean LEAN_READ = true;

    /**
     * Where the TURRET AXIS is relative to the Pinpoint tracking point (robot
     * rotation center), inches: forward = +, left = +. The camera bearing is
     * measured from the turret axis, so ignoring this offset puts a bearing
     * error of about atan(offset/distance) into any odometry-aided aiming
     * (4 in at 60 in range = 3.8 deg). MEASURE THIS with a tape measure.
     */
    public static final double TURRET_PIVOT_FORWARD_IN = 0.0;
    public static final double TURRET_PIVOT_LEFT_IN = 0.0;

    /** Ticks per mm in force: Pedro's ticksPerUnit override if set, else the factory value. */
    public static double activeTicksPerMm() {
        java.util.OptionalDouble o = Constants.localizerConfig.ticksPerUnit.get();
        if (o == null || !o.isPresent()) return FACTORY_TICKS_PER_MM;
        DistanceUnit unit = Constants.localizerConfig.encoderResolutionUnit.get();
        double mmPerUnit = DistanceUnit.MM.fromUnit(unit, 1.0);
        return o.getAsDouble() / mmPerUnit;
    }
}
