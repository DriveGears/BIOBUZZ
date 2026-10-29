package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/** Controls OUTTAKE RPM using software PIDF in RUN_WITHOUT_ENCODER mode and runs intake/transfer. */
public class shooterSubsystem {

    // ========================= VALUES TO CALIBRATE =========================

    /** Encoder ticks per motor revolution. Common bare goBILDA/REV encoder value is 28. */
    public static final double MOTOR_TICKS_PER_REVOLUTION = 28.0;

    /** Motor revolutions per OUTTAKE revolution. Use 1.0 for a direct drive. */
    public static final double MOTOR_REVS_PER_OUTTAKE_REV = 1.0;

    /** Change either sign if an intake mechanism runs backward. */
    public static final double INTAKE_POWER = 1.0;
    public static final double TRANSFER_POWER = 1.0;

    /** Distances and RPMs are paired. Measure new values for this year's mechanism. */
    public static final double[] SHOT_DISTANCE_IN = {
            38, 40, 42, 58, 60, 82, 90, 92, 98, 120, 132
    };
    public static final double[] SHOT_RPM = {
            1500, 1500, 1565, 1810, 1855, 2070, 2170, 2185, 2280, 2615, 2680
    };

    // Software PIDF Gains (RUN_WITHOUT_ENCODER motor mode)
    private static final double KP = 0.0020;
    private static final double KI = 0.0005;
    private static final double KD = 0.0001;
    private static final double KF = 0.000435;
    private static final double MAX_INTEGRAL_SUM = 200.0;

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
    private double targetRpm = SHOT_RPM[0];
    private double commandedRpm;
    private double distanceBucketIn = Double.NaN;
    private int readyLoops;
    private boolean atSpeed;

    private double integralSum = 0.0;
    private double lastError = 0.0;

    public shooterSubsystem(Hardware robot) {
        if (SHOT_DISTANCE_IN.length != SHOT_RPM.length || SHOT_DISTANCE_IN.length == 0) {
            throw new IllegalStateException("SHOT_DISTANCE_IN and SHOT_RPM must have equal lengths");
        }

        this.robot = robot;
        robot.outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stopMotors();
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        robot.intake.setPower(enabled ? INTAKE_POWER : 0);
        robot.transfer.setPower(enabled ? TRANSFER_POWER : 0);

        if (!enabled) {
            commandedRpm = 0;
            readyLoops = 0;
            atSpeed = false;
            robot.outtake.setPower(0);
            resetPID();
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    /** Uses the calibration table to select an RPM for a measured/predicted distance. */
    public void setTargetForDistance(double distanceIn) {
        if (Double.isNaN(distanceBucketIn)
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
        if (!enabled) return;

        double dt = clamp(dtSeconds, 0.001, 0.100);
        double slewRate = targetRpm >= commandedRpm
                ? SPIN_UP_RPM_PER_SECOND : SPIN_DOWN_RPM_PER_SECOND;
        double maxChange = slewRate * dt;
        commandedRpm += clamp(targetRpm - commandedRpm, -maxChange, maxChange);

        double targetTPS = rpmToTicksPerSecond(commandedRpm);
        double actualTPS = robot.outtake.getVelocity();
        double errorTPS = targetTPS - actualTPS;

        double pTerm = KP * errorTPS;
        integralSum += errorTPS * dt;
        integralSum = Math.max(-MAX_INTEGRAL_SUM, Math.min(MAX_INTEGRAL_SUM, integralSum));
        double iTerm = KI * integralSum;

        double derivative = (errorTPS - lastError) / dt;
        double dTerm = KD * derivative;

        double fTerm = KF * targetTPS;

        double motorPower = pTerm + iTerm + dTerm + fTerm;
        motorPower = clamp(motorPower, -1.0, 1.0);
        robot.outtake.setPower(motorPower);
        lastError = errorTPS;

        double actualRpm = getActualRpm();
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

    public void stop() {
        enabled = false;
        stopMotors();
    }

    private void stopMotors() {
        robot.outtake.setPower(0);
        robot.intake.setPower(0);
        robot.transfer.setPower(0);
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
}
