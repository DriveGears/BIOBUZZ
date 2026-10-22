package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware;

/** Controls flywheel RPM and runs the intake and transfer together with the shooter. */
public class shooterSubsystem {

    // ========================= VALUES TO CALIBRATE =========================

    /** Encoder ticks per motor revolution. Common bare goBILDA/REV encoder value is 28. */
    public static final double MOTOR_TICKS_PER_REVOLUTION = 28.0;

    /** Motor revolutions per flywheel revolution. Use 1.0 for a direct drive. */
    public static final double MOTOR_REVS_PER_FLYWHEEL_REV = 1.0;

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

    /** SDK velocity PIDF gains. Re-tune these if the flywheel motor/gearing changed. */
    public static final double FLYWHEEL_P = 10.0;
    public static final double FLYWHEEL_I = 0.08;
    public static final double FLYWHEEL_D = 2.0;
    public static final double FLYWHEEL_F = 14.0;

    /** Maximum allowed request and how quickly the commanded RPM may change. */
    public static final double MAX_RPM = 3400.0;
    public static final double SPIN_UP_RPM_PER_SECOND = 10700.0;
    public static final double SPIN_DOWN_RPM_PER_SECOND = 860.0;

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

    public shooterSubsystem(Hardware robot) {
        if (SHOT_DISTANCE_IN.length != SHOT_RPM.length || SHOT_DISTANCE_IN.length == 0) {
            throw new IllegalStateException("SHOT_DISTANCE_IN and SHOT_RPM must have equal lengths");
        }

        this.robot = robot;
        robot.flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.flywheel.setVelocityPIDFCoefficients(
                FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);
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
            robot.flywheel.setVelocity(0);
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

        double actualRpm = getActualRpm();
        robot.flywheel.setVelocity(rpmToTicksPerSecond(commandedRpm));

        /*
         * OPTIONAL OPEN-LOOP BOOST FROM THE OLD ROBOT (currently disabled)
         *
         * The old code temporarily switched to RUN_WITHOUT_ENCODER and full power whenever the
         * flywheel was far below its target. Keep normal setVelocity() control until testing proves
         * boost is necessary. To restore it, replace the setVelocity() line above with this logic:
         *
         * double rpmGap = targetRpm - actualRpm;
         * if (rpmGap > 320 && targetRpm > 110) {
         *     robot.flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         *     robot.flywheel.setPower(1.0);
         * } else {
         *     robot.flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         *     robot.flywheel.setVelocity(rpmToTicksPerSecond(commandedRpm));
         * }
         *
         * Add hysteresis before competition use so it does not switch modes every loop near 320 RPM.
         */

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
        return ticksPerSecondToRpm(robot.flywheel.getVelocity());
    }

    public double getDistanceBucketIn() {
        return distanceBucketIn;
    }

    public void stop() {
        enabled = false;
        stopMotors();
    }

    private void stopMotors() {
        robot.flywheel.setVelocity(0);
        robot.intake.setPower(0);
        robot.transfer.setPower(0);
    }

    private double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_REVS_PER_FLYWHEEL_REV
                * MOTOR_TICKS_PER_REVOLUTION / 60.0;
    }

    private double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond * 60.0
                / (MOTOR_TICKS_PER_REVOLUTION * MOTOR_REVS_PER_FLYWHEEL_REV);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
