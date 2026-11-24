package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Hardware;

/** Controls flywheel RPM using software PIDF in RUN_WITHOUT_ENCODER mode. */
public class shooterSubsystem {

    // ========================= VALUES TO CALIBRATE =========================

    /** Encoder ticks per motor revolution. Common bare goBILDA/REV encoder value is 28. */
    public static final double MOTOR_TICKS_PER_REVOLUTION = 28.0;

    /** Motor revolutions per OUTTAKE revolution. Use 1.0 for a direct drive. */
    public static final double MOTOR_REVS_PER_OUTTAKE_REV = 1.0;

    /** Normal field baseline used as a reference for the calibrated table below. */
    public static final double COMPETITION_RPM = 2900.0;

    /**
     * Exponential-like calibration: RPM rises gently through normal shooting
     * positions, then the slope increases toward the far corners. Linear interpolation
     * between these points approximates the curve without adding another tuning model.
     */
    public static final double[] SHOT_DISTANCE_IN = {
            90, 100, 130, 160
    };
    public static final double[] SHOT_RPM = {
            2900, 3000, 3000, 3050
    };

    // Competition PIDF defaults (RUN_WITHOUT_ENCODER motor mode).
    public static final double DEFAULT_KP = 0.003; // 0.0014
    /** Small trim only; a larger integral caused a +100 RPM rebound after each shot. */
    public static final double DEFAULT_KI = 0.00015;
    public static final double DEFAULT_KD = 0.0; // 0.0001
    /**
     * Log-calibrated at 2900 RPM: 0.000430 settled around 2931 RPM and contributed
     * to the post-shot rebound. P supplies the temporary recovery torque.
     */
    public static final double DEFAULT_KF = 0.000395;
    /** Integral only corrects small steady errors; it must not build during spin-up. */
    public static final double INTEGRAL_ZONE_RPM = 60.0;
    private static final double MAX_INTEGRAL_SUM = 150.0;

    /** Maximum allowed request and how quickly the commanded RPM may change. */
    public static final double MAX_RPM = 3700.0;
    public static final double SPIN_UP_RPM_PER_SECOND = 15000.0;
    public static final double SPIN_DOWN_RPM_PER_SECOND = 10000.0;

    /** Ready requires this RPM accuracy for READY_STABLE_LOOPS consecutive loops. */
    public static final double READY_TOLERANCE_RPM = 32.0;
    public static final int READY_STABLE_LOOPS = 8;

    /** Prevents tiny distance changes from constantly changing the requested RPM. */
    public static final double DISTANCE_BUCKET_IN = 2.0;
    public static final double DISTANCE_HYSTERESIS_IN = 1.5;

    // ========================= STATE =========================

    private final Hardware robot;
    private boolean enabled;
    private double targetRpm = COMPETITION_RPM;
    private double commandedRpm;
    private double distanceBucketIn = Double.NaN;
    private int readyLoops;
    private boolean atSpeed;

    private double integralSum = 0.0;
    private double lastError = 0.0;
    private double kP = DEFAULT_KP;
    private double kI = DEFAULT_KI;
    private double kD = DEFAULT_KD;
    private double kF = DEFAULT_KF;

    private double pTerm;
    private double iTerm;
    private double dTerm;
    private double fTerm;
    private double appliedPower;
    public shooterSubsystem(Hardware robot) {
        if (SHOT_DISTANCE_IN.length != SHOT_RPM.length || SHOT_DISTANCE_IN.length == 0) {
            throw new IllegalStateException("SHOT_DISTANCE_IN and SHOT_RPM must have equal lengths");
        }

        this.robot = robot;
        robot.outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stopFlywheel();
    }

    public void setShooterEnabled(boolean enabled) {
        if (this.enabled == enabled) return;
        this.enabled = enabled;

        if (!enabled) {
            commandedRpm = 0;
            readyLoops = 0;
            atSpeed = false;
            stopFlywheel();
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    /** Uses the calibration table to select an RPM for a measured/predicted distance. */
    public void setTargetForDistance(double distanceIn) {
        if (!Double.isFinite(distanceIn) || distanceIn <= 0) {
            distanceBucketIn = Double.NaN;
            setTargetRpm(COMPETITION_RPM);
            return;
        }

        if (!Double.isFinite(distanceBucketIn)
                || Math.abs(distanceIn - distanceBucketIn) > DISTANCE_HYSTERESIS_IN) {
            distanceBucketIn = Math.round(distanceIn / DISTANCE_BUCKET_IN)
                    * DISTANCE_BUCKET_IN;
        }
        setTargetRpm(rpmForDistance(distanceBucketIn));
    }

    /** Useful for test tuning without the camera. */
    public void setTargetRpm(double rpm) {
        targetRpm = clamp(rpm, 0, MAX_RPM);
    }

    /** Linear interpolation between measured points in SHOT_DISTANCE_IN/SHOT_RPM. */
    public double rpmForDistance(double distanceIn) {
        int last = SHOT_DISTANCE_IN.length - 1;
        if (distanceIn <= SHOT_DISTANCE_IN[0]) return SHOT_RPM[0];
        if (distanceIn >= SHOT_DISTANCE_IN[last]) return SHOT_RPM[last];

        for (int i = 0; i < last; i++) {
            double nearDistance = SHOT_DISTANCE_IN[i];
            double farDistance = SHOT_DISTANCE_IN[i + 1];
            if (distanceIn >= nearDistance && distanceIn <= farDistance) {
                double fraction = (distanceIn - nearDistance)
                        / (farDistance - nearDistance);
                return SHOT_RPM[i] + fraction * (SHOT_RPM[i + 1] - SHOT_RPM[i]);
            }
        }
        return SHOT_RPM[last];
    }

    /** Call once per OpMode loop. */
    public void update(double dtSeconds) {
        if (!enabled) {
            return;
        }

        double dt = clamp(dtSeconds, 0.001, 0.100);
        double slewRate = targetRpm >= commandedRpm
                ? SPIN_UP_RPM_PER_SECOND : SPIN_DOWN_RPM_PER_SECOND;
        double maxChange = slewRate * dt;
        commandedRpm += clamp(targetRpm - commandedRpm, -maxChange, maxChange);

        double targetTPS = rpmToTicksPerSecond(commandedRpm);
        double actualTPS = robot.outtake.getVelocity();
        double errorTPS = targetTPS - actualTPS;
        double errorRpm = ticksPerSecondToRpm(errorTPS);

        // Do not carry positive recovery torque past the setpoint. Flywheel inertia
        // already continues accelerating briefly after the error changes sign.
        if (errorTPS * lastError < 0.0) {
            integralSum = 0.0;
        }

        pTerm = kP * errorTPS;
        double derivative = (errorTPS - lastError) / dt;
        dTerm = kD * derivative;
        fTerm = kF * targetTPS;

        // Reset stored torque outside the narrow steady-state zone. Inside the
        // zone, accept integration only when it does not push farther into a
        // saturated output. This removes the slow +RPM settle after spin-up.
        if (Math.abs(errorRpm) > INTEGRAL_ZONE_RPM) {
            integralSum = 0.0;
        } else {
            double candidateIntegralSum = clamp(
                    integralSum + errorTPS * dt,
                    -MAX_INTEGRAL_SUM, MAX_INTEGRAL_SUM);
            double candidateITerm = kI * candidateIntegralSum;
            double candidatePower = pTerm + candidateITerm + dTerm + fTerm;
            boolean pushesHighSaturation = candidatePower > 1.0 && errorTPS > 0;
            boolean pushesLowSaturation = candidatePower < 0.0 && errorTPS < 0;
            if (!pushesHighSaturation && !pushesLowSaturation) {
                integralSum = candidateIntegralSum;
            }
        }
        iTerm = kI * integralSum;

        double motorPower = pTerm + iTerm + dTerm + fTerm;
        // A flywheel only needs forward power. Negative PID output would actively brake it.
        appliedPower = clamp(motorPower, 0.0, 1.0);
        robot.outtake.setPower(appliedPower);
        lastError = errorTPS;

        double actualRpm = ticksPerSecondToRpm(actualTPS);
        if (Math.abs(actualRpm - targetRpm) <= READY_TOLERANCE_RPM) {
            readyLoops++;
        } else {
            readyLoops = 0;
        }
        atSpeed = readyLoops >= READY_STABLE_LOOPS;
    }

    public boolean isAtSpeed() {
        return atSpeed;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getCommandedRpm() {
        return commandedRpm;
    }

    public double getActualRpm() {
        return ticksPerSecondToRpm(robot.outtake.getVelocity());
    }

    public double getDistanceBucketIn() {
        return distanceBucketIn;
    }

    /** Changes this instance's gains immediately; intended for the temporary tuning OpMode. */
    public void setPidfGains(double kP, double kI, double kD, double kF) {
        this.kP = nonNegativeFinite(kP);
        this.kI = nonNegativeFinite(kI);
        this.kD = nonNegativeFinite(kD);
        this.kF = nonNegativeFinite(kF);
        resetPID();
    }

    public void resetPidfGains() {
        setPidfGains(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_KF);
    }

    public double getKp() {
        return kP;
    }

    public double getKi() {
        return kI;
    }

    public double getKd() {
        return kD;
    }

    public double getKf() {
        return kF;
    }

    public double getPTerm() {
        return pTerm;
    }

    public double getITerm() {
        return iTerm;
    }

    public double getDTerm() {
        return dTerm;
    }

    public double getFTerm() {
        return fTerm;
    }

    public double getAppliedPower() {
        return appliedPower;
    }

    /** Adds the complete controller state to a flight-recorder row for PIDF tuning. */
    public void logTo(FlightRecorder recorder) {
        double actualRpm = getActualRpm();
        recorder.put("shoot_enabled", enabled);
        recorder.put("shoot_target_rpm", targetRpm);
        recorder.put("shoot_distance_bucket_in", distanceBucketIn);
        recorder.put("shoot_cmd_rpm", commandedRpm);
        recorder.put("shoot_actual_rpm", actualRpm);
        recorder.put("shoot_error_rpm", targetRpm - actualRpm);
        recorder.put("shoot_at_speed", atSpeed);
        recorder.put("shoot_power", appliedPower);
        recorder.put("shoot_p", pTerm);
        recorder.put("shoot_i", iTerm);
        recorder.put("shoot_d", dTerm);
        recorder.put("shoot_f", fTerm);
        recorder.put("shoot_kp", kP);
        recorder.put("shoot_ki", kI);
        recorder.put("shoot_kd", kD);
        recorder.put("shoot_kf", kF);
    }

    public void stop() {
        enabled = false;
        commandedRpm = 0;
        readyLoops = 0;
        atSpeed = false;
        stopFlywheel();
    }

    private void stopFlywheel() {
        robot.outtake.setPower(0);
        pTerm = 0;
        iTerm = 0;
        dTerm = 0;
        fTerm = 0;
        appliedPower = 0;
        resetPID();
    }

    private void resetPID() {
        integralSum = 0.0;
        lastError = 0.0;
    }

    private double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_REVS_PER_OUTTAKE_REV
                * MOTOR_TICKS_PER_REVOLUTION / 60.0;
    }

    private double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond * 60.0
                / (MOTOR_TICKS_PER_REVOLUTION * MOTOR_REVS_PER_OUTTAKE_REV);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double nonNegativeFinite(double value) {
        return Double.isFinite(value) ? Math.max(0, value) : 0;
    }

}
