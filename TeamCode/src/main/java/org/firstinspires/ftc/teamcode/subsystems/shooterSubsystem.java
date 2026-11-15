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

    /** Competition shooter speed. Distance-based RPM selection is intentionally disabled. */
    public static final double COMPETITION_RPM = 2900.0;

    /** Distances and RPMs are paired. Measure new values for this year's mechanism. */
    public static final double[] SHOT_DISTANCE_IN = {
            38, 40, 42, 58, 60, 82, 90, 92, 98, 120, 132
    };
    public static final double[] SHOT_RPM = {
            1500, 1500, 1565, 1810, 1855, 2070, 2170, 2185, 2280, 2615, 2680
    };

    // Competition PIDF defaults (RUN_WITHOUT_ENCODER motor mode).
    public static final double DEFAULT_KP = 0.0014; // 0.003
    public static final double DEFAULT_KI = 0.0005;
    public static final double DEFAULT_KD = 0.0; // 0.0001
    public static final double DEFAULT_KF = 0.0005; // 0.000435
    private static final double MAX_INTEGRAL_SUM = 200.0;

    // Full-range servo positions for goBILDA RGB Indicator Light 3118-0808-0002.
    private static final double LED_RED_POSITION = 0.300;   // 1100 us
    private static final double LED_GREEN_POSITION = 0.500; // 1500 us

    /** Maximum allowed request and how quickly the commanded RPM may change. */
    public static final double MAX_RPM = 3400.0;
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
    private IndicatorState indicatorState;

    private enum IndicatorState {
        OFF,
        SPINNING_UP,
        READY
    }

    public shooterSubsystem(Hardware robot) {
        if (SHOT_DISTANCE_IN.length != SHOT_RPM.length || SHOT_DISTANCE_IN.length == 0) {
            throw new IllegalStateException("SHOT_DISTANCE_IN and SHOT_RPM must have equal lengths");
        }

        this.robot = robot;
        robot.outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stopFlywheel();
        setIndicator(IndicatorState.OFF);
    }

    public void setShooterEnabled(boolean enabled) {
        if (this.enabled == enabled) return;
        this.enabled = enabled;

        if (!enabled) {
            commandedRpm = 0;
            readyLoops = 0;
            atSpeed = false;
            stopFlywheel();
            setIndicator(IndicatorState.OFF);
        } else {
            setIndicator(IndicatorState.SPINNING_UP);
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    /** Uses the calibration table to select an RPM for a measured/predicted distance. */
    public void setTargetForDistance(double distanceIn) {
        distanceBucketIn = distanceIn;
        setTargetRpm(COMPETITION_RPM);
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
        if (!enabled) return;

        double dt = clamp(dtSeconds, 0.001, 0.100);
        double slewRate = targetRpm >= commandedRpm
                ? SPIN_UP_RPM_PER_SECOND : SPIN_DOWN_RPM_PER_SECOND;
        double maxChange = slewRate * dt;
        commandedRpm += clamp(targetRpm - commandedRpm, -maxChange, maxChange);

        double targetTPS = rpmToTicksPerSecond(commandedRpm);
        double actualTPS = robot.outtake.getVelocity();
        double errorTPS = targetTPS - actualTPS;

        pTerm = kP * errorTPS;
        integralSum += errorTPS * dt;
        integralSum = Math.max(-MAX_INTEGRAL_SUM, Math.min(MAX_INTEGRAL_SUM, integralSum));
        iTerm = kI * integralSum;

        double derivative = (errorTPS - lastError) / dt;
        dTerm = kD * derivative;

        fTerm = kF * targetTPS;

        double motorPower = pTerm + iTerm + dTerm + fTerm;
        // A flywheel only needs forward power. Negative PID output would actively brake it.
        appliedPower = clamp(motorPower, 0.0, 1.0);
        robot.outtake.setPower(appliedPower);
        lastError = errorTPS;

        double actualRpm = getActualRpm();
        if (Math.abs(actualRpm - targetRpm) <= READY_TOLERANCE_RPM) {
            readyLoops++;
        } else {
            readyLoops = 0;
        }
        atSpeed = readyLoops >= READY_STABLE_LOOPS;
        setIndicator(atSpeed ? IndicatorState.READY : IndicatorState.SPINNING_UP);
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

    public void stop() {
        enabled = false;
        commandedRpm = 0;
        readyLoops = 0;
        atSpeed = false;
        stopFlywheel();
        setIndicator(IndicatorState.OFF);
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

    private void setIndicator(IndicatorState newState) {
        if (indicatorState == newState) return;
        indicatorState = newState;

        if (newState == IndicatorState.OFF) {
            robot.shooterLed.setPwmDisable();
            return;
        }

        robot.shooterLed.setPwmEnable();
        robot.shooterLed.setPosition(newState == IndicatorState.READY
                ? LED_GREEN_POSITION : LED_RED_POSITION);
    }
}
