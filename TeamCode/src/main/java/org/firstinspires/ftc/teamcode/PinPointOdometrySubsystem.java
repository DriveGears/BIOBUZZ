package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Threaded PinPointOdometrySubsystem — v2.
 *
 * Changes from v1:
 *   1. STATIC WORKER GUARD: a previous run's PinpointReader can no longer
 *      survive into this run. The constructor force-stops any worker left
 *      over from an earlier instance (e.g. when the old OpMode crashed and
 *      its cleanup never ran). Two workers from two runs hammering the same
 *      I2C bus through different driver objects was wedging the hub.
 *   2. lastReadMs is now stamped ONLY on a VALID (non-NaN) read. Previously
 *      a Pinpoint returning continuous NaNs kept isStale() == false while
 *      the pose drifted on pure dead-reckoning, so the TeleOp never fell
 *      back to robot-centric driving.
 *
 * The Pinpoint read (update() + getters) runs on a worker thread and
 * publishes pose/velocity into volatile fields. The control loop reads only
 * cached fields via getX/getY/getHeading — which never block.
 */
public class PinPointOdometrySubsystem {

    private static final long STALE_MS        = 100;   // cache older than this => odo unreliable
    private static final long WORKER_SLEEP_MS = 5;     // ~200 Hz read cadence

    // Guard against a leaked worker from a previous OpMode run.
    private static volatile Thread  previousWorker  = null;
    private static volatile boolean previousRunning = false; // not directly usable; see note below

    private final GoBildaPinpointDriver pinpointDriver;
    private final Object driverLock = new Object();

    // ---- published by worker, read by main loop ----
    private volatile double x = 0, y = 0, heading = 0;   // cm, cm, rad
    private volatile double vx = 0, vy = 0, vtheta = 0;  // mm/s, mm/s, rad/s
    private volatile int    rawX = 0, rawY = 0;
    private volatile long   lastReadMs  = 0;
    private volatile int    nanCounter  = 0;
    private volatile double workerLoopMs = 0;

    // ---- worker-thread-only dead-reckon state ----
    private double previousX = 0, previousY = 0, previousHeading = 0;

    private final ElapsedTime totalTimer    = new ElapsedTime();
    private final ElapsedTime workerDtTimer = new ElapsedTime();

    private Thread worker;
    private volatile boolean running = false;

    // ---- logging (worker thread only) ----
    private BufferedWriter      logWriter      = null;
    private volatile boolean    loggingEnabled = false;
    private final StringBuilder logLine        = new StringBuilder(256);
    private int                 logFailCount   = 0;
    private int                 frameCount     = 0;
    private double prevLogX = 0, prevLogY = 0, prevLogHeading = 0;

    public PinPointOdometrySubsystem(Hardware hw) {

        // ============================================================
        // KILL ANY LEAKED WORKER FROM A PREVIOUS RUN before touching
        // the bus. interrupt() can't unblock native I2C, but it stops
        // the loop at the next sleep/iteration; we then wait briefly
        // for it to die before starting our own reads.
        // ============================================================
        Thread old = previousWorker;
        if (old != null && old.isAlive()) {
            RobotLog.ww("PinpointOdo", "Leaked PinpointReader from previous run detected — stopping it.");
            old.interrupt();
            try { old.join(500); } catch (InterruptedException ignored) {}
            if (old.isAlive()) {
                // Hard-wedged in native I2C. We can't kill it; log loudly so
                // a freeze this run is attributable.
                RobotLog.ee("PinpointOdo", "Old PinpointReader did NOT die — possible I2C contention this run!");
            }
        }

        pinpointDriver = hw.pinPointOdo;

        pinpointDriver.setOffsets(0, 20);
        pinpointDriver.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpointDriver.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpointDriver.resetPosAndIMU();

        totalTimer.reset();
        workerDtTimer.reset();

        try {
            String ts   = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            String path = "/sdcard/FIRST/odo_log_" + ts + ".csv";
            logWriter   = new BufferedWriter(new FileWriter(path), 32768);
            loggingEnabled = true;
            logWriter.write(
                    "time_ms,x_cm,y_cm,heading_rad,heading_deg," +
                            "raw_encoder_x,raw_encoder_y," +
                            "vel_x,vel_y,heading_vel," +
                            "nan_count,using_dead_reckon,loop_dt_ms," +
                            "delta_x_cm,delta_y_cm,delta_heading_rad," +
                            "x_jump_flag,h_jump_flag\n");
            logWriter.flush();
        } catch (IOException e) {
            loggingEnabled = false;
        }

        startWorker();
    }

    // =====================================================================
    // Worker thread — the ONLY place the Pinpoint is read
    // =====================================================================

    private void startWorker() {
        running    = true;
        lastReadMs = System.currentTimeMillis();
        worker = new Thread(this::workerLoop, "PinpointReader");
        worker.setDaemon(true);
        worker.start();
        previousWorker = worker;   // register so the NEXT run can clean us up
    }

    public void stopThread() {
        running = false;
        if (worker != null) worker.interrupt();
        if (worker == previousWorker) previousWorker = null;
        closeLog();
    }

    private void workerLoop() {
        while (running && !Thread.currentThread().isInterrupted()) {
            double dtMs = workerDtTimer.milliseconds();
            workerDtTimer.reset();
            workerLoopMs = dtMs;

            boolean usedDeadReckon = false;
            try {
                synchronized (driverLock) {
                    pinpointDriver.update();   // blocking I2C — but on THIS thread, not the loop

                    double cx = pinpointDriver.getPosX();
                    double cy = pinpointDriver.getPosY();
                    double ch = pinpointDriver.getHeading();

                    if (Double.isNaN(cx) || Double.isNaN(cy) || Double.isNaN(ch)) {
                        nanCounter++;
                        usedDeadReckon = true;
                        double dtSec = dtMs / 1000.0;
                        x       = previousX       + (vx / 10.0) * dtSec;
                        y       = previousY       + (vy / 10.0) * dtSec;
                        heading = previousHeading + vtheta * dtSec;
                        previousX = x; previousY = y; previousHeading = heading;
                        // NOTE: lastReadMs is NOT stamped — continuous NaNs
                        // must trip isStale() so the TeleOp drops to
                        // robot-centric instead of trusting drifted pose.
                    } else {
                        x       = cx / 10.0;
                        y       = cy / 10.0;      // +Y convention (driver-native)
                        heading = ch;             // radians — do not convert
                        previousX = x; previousY = y; previousHeading = heading;

                        vx     = pinpointDriver.getVelX();
                        vy     = pinpointDriver.getVelY();
                        vtheta = pinpointDriver.getHeadingVelocity();
                        rawX   = pinpointDriver.getEncoderX();
                        rawY   = pinpointDriver.getEncoderY();

                        lastReadMs = System.currentTimeMillis();  // VALID reads only
                    }
                }
                if (loggingEnabled) writeLog(dtMs, usedDeadReckon);
            } catch (Exception e) {
                // Driver threw — keep the cached pose; staleness will trip if it persists.
            }

            try { Thread.sleep(WORKER_SLEEP_MS); }
            catch (InterruptedException e) { break; }
        }
    }

    private void writeLog(double dtMs, boolean usedDeadReckon) {
        if (logWriter == null) return;
        frameCount++;

        double deltaX = x - prevLogX;
        double deltaY = y - prevLogY;
        double deltaH = heading - prevLogHeading;
        while (deltaH >  Math.PI) deltaH -= 2 * Math.PI;
        while (deltaH < -Math.PI) deltaH += 2 * Math.PI;

        int xJump = Math.abs(deltaX) > 50.0 ? 1 : 0;
        int hJump = Math.abs(deltaH) > 1.0  ? 1 : 0;

        prevLogX = x; prevLogY = y; prevLogHeading = heading;

        try {
            logLine.setLength(0);
            logLine.append(totalTimer.milliseconds()).append(',')
                    .append(x).append(',')
                    .append(y).append(',')
                    .append(heading).append(',')
                    .append(Math.toDegrees(heading)).append(',')
                    .append(rawX).append(',')
                    .append(rawY).append(',')
                    .append(vx).append(',')
                    .append(vy).append(',')
                    .append(vtheta).append(',')
                    .append(nanCounter).append(',')
                    .append(usedDeadReckon ? 1 : 0).append(',')
                    .append(dtMs).append(',')
                    .append(deltaX).append(',')
                    .append(deltaY).append(',')
                    .append(deltaH).append(',')
                    .append(xJump).append(',')
                    .append(hJump).append('\n');
            logWriter.write(logLine.toString());
            if (frameCount % 100 == 0) logWriter.flush();
        } catch (IOException e) {
            logFailCount++;
            if (logFailCount > 20) loggingEnabled = false;
        }
    }

    // =====================================================================
    // Logging control
    // =====================================================================

    public void disableLogging() {
        loggingEnabled = false;
        if (logWriter != null) {
            try { logWriter.flush(); logWriter.close(); } catch (IOException ignored) {}
            logWriter = null;
        }
    }

    public void closeLog() {
        if (logWriter != null) {
            try { logWriter.flush(); logWriter.close(); } catch (IOException ignored) {}
            logWriter = null;
        }
        loggingEnabled = false;
    }

    // No-op kept for API compatibility — the worker reads continuously now.
    public void processOdometry() { /* intentionally empty */ }

    // =====================================================================
    // Position writes — synchronized against the worker's reads
    // =====================================================================

    public void setNewPosition(double xCm, double yCm, double headingDeg) {
        synchronized (driverLock) {
            pinpointDriver.setPosition(
                    new Pose2D(DistanceUnit.CM, xCm, yCm, AngleUnit.DEGREES, headingDeg));
        }
    }

    public void reset() {
        synchronized (driverLock) {
            pinpointDriver.resetPosAndIMU();
        }
        prevLogX = 0; prevLogY = 0; prevLogHeading = 0;
    }

    public void resetPositionOnly() {
        synchronized (driverLock) {
            double currentHeadingDeg = Math.toDegrees(heading);
            pinpointDriver.setPosition(
                    new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, currentHeadingDeg));
        }
        prevLogX = 0; prevLogY = 0;
    }

    // =====================================================================
    // Cached getters — never block
    // =====================================================================

    /** True if the worker hasn't produced a fresh VALID read recently. */
    public boolean isStale() { return (System.currentTimeMillis() - lastReadMs) > STALE_MS; }

    public int    getNanCounter()      { return nanCounter; }
    public double getControlLoopTime() { return workerLoopMs / 1000.0; }
    public double getX()               { return x; }
    public double getY()               { return y; }
    public double getHeading()         { return heading; }   // radians
    public double getHeadingVelocity() { return vtheta; }    // rad/s (cached)
    public double getVx()              { return vx; }
    public double getVy()              { return vy; }
    public double getVtheta()          { return vtheta; }
    public double getRawX()            { return rawX; }
    public double getRawY()            { return rawY; }
}