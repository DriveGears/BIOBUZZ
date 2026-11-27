package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;
import java.util.Locale;
import java.util.OptionalDouble;

public class Hardware {

    public static final double SHOOTER_LED_PWM_MIN_US = 600.0;
    public static final double SHOOTER_LED_PWM_MAX_US = 2400.0;

    private static Hardware instance;
    public final DcMotorEx lf;
    public final DcMotorEx rf;
    public final DcMotorEx lb;
    public final DcMotorEx rb;
    public final DcMotorEx outtake;
    public final DcMotorEx intake;
    public final DcMotorEx transfer;
    public final DcMotorEx turret;
    public final ServoImplEx shooterLed;

    private final List<LynxModule> hubs;
    private VoltageSensor batterySensor;
    private boolean manualBulk;
    private boolean odoLean;
    private int odoFirmware = -1;
    private final StringBuilder odoReport = new StringBuilder();

    public final GoBildaPinpointDriver odo;
    public Limelight3A limelight = null;

    public Hardware(HardwareMap hwMap) {
        this(hwMap, true);
    }

    public Hardware(HardwareMap hwMap, boolean includeLimelight) {
        // Always restore live reads in case an earlier OpMode left the hubs in
        // MANUAL bulk-caching mode without clearing them each loop.
        hubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            hub.clearBulkCache();
        }

        this.lf = hwMap.get(DcMotorEx.class, Specifications.FTLF_MOTOR);
        this.rf = hwMap.get(DcMotorEx.class, Specifications.FTRT_MOTOR);
        this.lb = hwMap.get(DcMotorEx.class, Specifications.BKLF_MOTOR);
        this.rb = hwMap.get(DcMotorEx.class, Specifications.BKRT_MOTOR);
        this.outtake = hwMap.get(DcMotorEx.class, Specifications.OUTTAKE);
        this.shooterLed = hwMap.get(ServoImplEx.class, Specifications.SHOOTER_LED);
        this.intake = hwMap.get(DcMotorEx.class, Specifications.INTAKE);
        this.transfer = hwMap.get(DcMotorEx.class, Specifications.TRANSFER);
        this.turret = hwMap.get(DcMotorEx.class, Specifications.TURRET);
        this.limelight = includeLimelight
                ? hwMap.get(Limelight3A.class, Specifications.LIMELIGHT)
                : null;
        this.odo = hwMap.get(GoBildaPinpointDriver.class,Specifications.ODO);

        lf.setDirection(DcMotorEx.Direction.REVERSE);
        lb.setDirection(DcMotorEx.Direction.REVERSE);
        rf.setDirection(DcMotorEx.Direction.FORWARD);
        rb.setDirection(DcMotorEx.Direction.FORWARD);
        outtake.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        transfer.setDirection(DcMotorEx.Direction.REVERSE);
        turret.setDirection(DcMotorEx.Direction.FORWARD);

        lf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // Let the flywheel coast when disabled; the shooter controller never commands reverse power.
        outtake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Keep the goBILDA RGB indicator on the FTC servo convention. Its useful
        // solid-color band is roughly 1100-1900 us; the wider 500-2500 us range
        // made low position values (especially red) fall into the OFF region.
        shooterLed.setPwmRange(new PwmControl.PwmRange(
                SHOOTER_LED_PWM_MIN_US, SHOOTER_LED_PWM_MAX_US));

        for (VoltageSensor sensor : hwMap.voltageSensor) {
            batterySensor = sensor;
            break;
        }

        // Position and velocity remain readable in this mode. The turret uses
        // its own software PID, so do not layer the hub's velocity PID on top.
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        try {
            configureOdometry();
        } catch (Throwable t) {
            odoReport.append("ODO_CONFIG_EXCEPTION ").append(t).append('\n');
        }
    }

    /**
     * Configures the Pinpoint exactly like Pedro does (offsets, resolution,
     * directions) plus yaw scalar, zeroes the IMU, and seeds the start pose.
     * The robot must be STATIONARY during INIT (IMU calibration ~0.3 s).
     * Everything found/done is appended to {@link #getOdometryReport()}.
     */
    private void configureOdometry() {
        if (odo == null) return;
        long t0 = System.nanoTime();

        // What was left over from the previous OpMode / power-up (diagnostic only).
        odo.update();
        odoReport.append(String.format(Locale.US,
                "ODO_STALE_STATE status=%s pose=(%.2f, %.2f, %.1fdeg) offsets=(%.3f, %.3f)in yaw=%.4f mmPerTick(read via enc)\n",
                odo.getDeviceStatus(), odo.getPosX(DistanceUnit.INCH), odo.getPosY(DistanceUnit.INCH),
                odo.getHeading(AngleUnit.DEGREES), odo.getXOffset(DistanceUnit.INCH),
                odo.getYOffset(DistanceUnit.INCH), odo.getYawScalar()));

        try {
            odoFirmware = odo.getDeviceVersion();
        } catch (Throwable t) {
            odoFirmware = -1;
        }
        odoReport.append("ODO_FIRMWARE version=").append(odoFirmware)
                .append(odoFirmware >= 3 ? " (bulk read scope supported)\n"
                        : " (bulk read scope NOT supported: using the driver's full read)\n");

        // Pod geometry/resolution come from Pedro's Constants (single source of truth).
        PinpointConfig pc = Constants.localizerConfig;
        odo.setOffsets(pc.xPodOffset.get(), pc.yPodOffset.get(), pc.offsetUnits.get());
        OptionalDouble ticks = pc.ticksPerUnit.get();
        if (ticks != null && ticks.isPresent()) {
            odo.setEncoderResolution(ticks.getAsDouble(), pc.encoderResolutionUnit.get());
        } else {
            odo.setEncoderResolution(pc.podType.get());
        }
        odo.setEncoderDirections(pc.xPodDirection.get(), pc.yPodDirection.get());
        odo.setYawScalar(OdometryConfig.YAW_SCALAR);

        odo.resetPosAndIMU();
        GoBildaPinpointDriver.DeviceStatus status = GoBildaPinpointDriver.DeviceStatus.NOT_READY;
        long deadline = System.nanoTime() + 1_500_000_000L;
        while (System.nanoTime() < deadline) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
            odo.update();
            status = odo.getDeviceStatus();
            if (status == GoBildaPinpointDriver.DeviceStatus.READY) break;
        }

        odo.setPosition(new Pose2D(DistanceUnit.INCH,
                OdometryConfig.START_X_IN, OdometryConfig.START_Y_IN,
                AngleUnit.DEGREES, OdometryConfig.START_HEADING_DEG));
        odo.update();
        double fx = odo.getPosX(DistanceUnit.INCH), fy = odo.getPosY(DistanceUnit.INCH);
        double fh = odo.getHeading(AngleUnit.DEGREES);

        // Lean read self-check: same pose with the smaller register set?
        odoLean = false;
        if (OdometryConfig.LEAN_READ && odoFirmware >= 3) {
            try {
            odo.setBulkReadScope(
                    GoBildaPinpointDriver.Register.X_POSITION, GoBildaPinpointDriver.Register.Y_POSITION,
                    GoBildaPinpointDriver.Register.H_ORIENTATION, GoBildaPinpointDriver.Register.X_VELOCITY,
                    GoBildaPinpointDriver.Register.Y_VELOCITY, GoBildaPinpointDriver.Register.H_VELOCITY);
            odo.update();
            double lx = odo.getPosX(DistanceUnit.INCH), ly = odo.getPosY(DistanceUnit.INCH);
            double lh = odo.getHeading(AngleUnit.DEGREES);
            double dh = Math.abs(((lh - fh) + 540) % 360 - 180);
            odoLean = Math.hypot(lx - fx, ly - fy) < 0.5 && dh < 1.0
                    && Math.abs(lx - OdometryConfig.START_X_IN) < 3.0;
            if (!odoLean) useFullOdometryScope();
            odoReport.append(String.format(Locale.US,
                    "ODO_LEAN_READ %s full=(%.2f,%.2f,%.1f) lean=(%.2f,%.2f,%.1f)\n",
                    odoLean ? "OK" : "REJECTED->full", fx, fy, fh, lx, ly, lh));
            } catch (Throwable t) {
                odoLean = false;
                useFullOdometryScope();
                odoReport.append("ODO_LEAN_READ FAILED ").append(t).append(" -> full read\n");
            }
        }
        odoReport.append(String.format(Locale.US,
                "ODO_CONFIGURED status=%s start=(%.2f,%.2f,%.1fdeg) offsets=(%.3f,%.3f)in ticksPerMm=%.4f yawScalar=%.5f took=%.0fms\n",
                status, fx, fy, fh, pc.xPodOffset.get(), pc.yPodOffset.get(),
                OdometryConfig.activeTicksPerMm(), OdometryConfig.YAW_SCALAR,
                (System.nanoTime() - t0) / 1e6));
        if (status != GoBildaPinpointDriver.DeviceStatus.READY) {
            odoReport.append("ODO_WARNING device not READY after init: ").append(status).append('\n');
        }
    }

    /**
     * Re-zeroes the IMU and re-seeds the start pose. Robot MUST be stationary (~0.4 s).
     * Used by the calibration OpMode; normal OpModes get this once at construction.
     */
    public String rezeroOdometry() {
        if (odo == null) return "no odo";
        odo.resetPosAndIMU();
        GoBildaPinpointDriver.DeviceStatus status = GoBildaPinpointDriver.DeviceStatus.NOT_READY;
        long deadline = System.nanoTime() + 1_500_000_000L;
        while (System.nanoTime() < deadline) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
            odo.update();
            status = odo.getDeviceStatus();
            if (status == GoBildaPinpointDriver.DeviceStatus.READY) break;
        }
        odo.setPosition(new Pose2D(DistanceUnit.INCH,
                OdometryConfig.START_X_IN, OdometryConfig.START_Y_IN,
                AngleUnit.DEGREES, OdometryConfig.START_HEADING_DEG));
        odo.update();
        return "status=" + status;
    }

    /** Restores the driver's complete register read (needed for raw encoder ticks / status). */
    public void useFullOdometryScope() {
        odoLean = false;
        if (odo == null || odoFirmware < 3) return;   // older firmware always does the full read
        try {
            odo.setBulkReadScope(
                    GoBildaPinpointDriver.Register.DEVICE_STATUS, GoBildaPinpointDriver.Register.LOOP_TIME,
                    GoBildaPinpointDriver.Register.X_ENCODER_VALUE, GoBildaPinpointDriver.Register.Y_ENCODER_VALUE,
                    GoBildaPinpointDriver.Register.X_POSITION, GoBildaPinpointDriver.Register.Y_POSITION,
                    GoBildaPinpointDriver.Register.H_ORIENTATION, GoBildaPinpointDriver.Register.X_VELOCITY,
                    GoBildaPinpointDriver.Register.Y_VELOCITY, GoBildaPinpointDriver.Register.H_VELOCITY);
        } catch (Throwable t) {
            odoReport.append("ODO_FULL_SCOPE_FAILED ").append(t).append('\n');
        }
    }

    public int getOdometryFirmware() {
        return odoFirmware;
    }

    public boolean isOdometryLean() {
        return odoLean;
    }

    /** Multi-line text of what init found and did; write it to the event log. */
    public String getOdometryReport() {
        return odoReport.toString();
    }

    /**
     * One hub read per loop instead of one per getter call. After this, the
     * OpMode MUST call {@link #beginLoop()} at the top of every loop.
     */
    public void enableManualBulkReads() {
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        manualBulk = true;
        beginLoop();
    }

    /** Refreshes the cached encoder/velocity data. No-op in AUTO mode. */
    public void beginLoop() {
        if (!manualBulk) return;
        for (LynxModule hub : hubs) hub.clearBulkCache();
    }

    public boolean isManualBulk() {
        return manualBulk;
    }

    /** These are separate hub commands (~1 ms each): sample slowly, never every loop. */
    public double readBatteryVolts() {
        return batterySensor == null ? Double.NaN : batterySensor.getVoltage();
    }

    public double readTurretCurrentAmps() {
        return turret.getCurrent(CurrentUnit.AMPS);
    }

    /** Separate hub command: sample periodically rather than every control loop. */
    public double readOuttakeCurrentAmps() {
        return outtake.getCurrent(CurrentUnit.AMPS);
    }

    public static Hardware getInstance(HardwareMap hwMap) {
        instance = new Hardware(hwMap);
        return instance;
    }

    /** Creates hardware without mapping Limelight for tests that do not use vision. */
    public static Hardware getInstance(HardwareMap hwMap, boolean includeLimelight) {
        instance = new Hardware(hwMap, includeLimelight);
        return instance;
    }

    public static void resetInstance() {
        instance = null;
    }

    /**
     * Defines the turret's current physical position as the center (zero).
     * Only call this while the turret is known to be mechanically centered.
     */
    public void setTurretCurrentPositionAsZero() {
        turret.setPower(0);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Returns hardware-relative ticks; zero survives normal OpMode transitions. */
    public int getTurretPositionTicks() {
        return turret.getCurrentPosition();
    }

    /** Exposed for diagnostics; control code should use getTurretPositionTicks(). */
    public int getTurretRawEncoderTicks() {
        return turret.getCurrentPosition();
    }

    public int getTurretEncoderZeroTicks() {
        return 0;
    }

}
