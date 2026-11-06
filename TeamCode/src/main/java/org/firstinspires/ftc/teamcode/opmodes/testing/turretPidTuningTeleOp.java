package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.turretSubsystem2;

/** Live turret tuning without rebuilding or redeploying between gain changes. */
@TeleOp(name = "turret pid tuning", group = "Testing")
public class turretPidTuningTeleOp extends LinearOpMode {

    private static final double DRIVE_SPEED = 0.55;
    private static final double MANUAL_TURRET_POWER = 0.18;

    private static final String[] PARAMETER_NAMES = {
            "Position Kp", "Velocity Kd", "Position Ki", "Velocity Kv",
            "Stiction", "Max integral", "Vision gain", "Aim tolerance",
            "Max turret power", "Odometry blend", "Fixed flight time",
            "TOF per inch", "Shot delay"
    };

    private static final double[] PARAMETER_STEPS = {
            0.001, 0.0005, 0.0001, 0.0001,
            0.005, 0.005, 0.01, 0.05,
            0.05, 0.05, 0.01, 0.0005, 0.01
    };

    private int selectedParameter;
    private boolean lastDpadUp;
    private boolean lastDpadDown;
    private boolean lastDpadLeft;
    private boolean lastDpadRight;
    private boolean lastPredictionButton;
    private boolean lastOdometryButton;
    private boolean lastLatencyButton;
    private boolean lastAllianceButton;
    private boolean lastZeroButton;

    @Override
    public void runOpMode() {
        Hardware.resetInstance();
        Hardware robot = Hardware.getInstance(hardwareMap);
        turretSubsystem2 turret = new turretSubsystem2(robot);
        ElapsedTime loopTimer = new ElapsedTime();

        telemetry.addLine("Turret PID Live Tuner ready");
        telemetry.addLine("G1: drive, RT auto-aim, Back + D-pad L/R manual turret");
        telemetry.addLine("G1 both stick clicks: zero turret at known center");
        telemetry.addLine("G1 D-pad U/D select, L/R change; LB fine, RB coarse");
        telemetry.addLine("G1 A prediction | B odometry | X latency | Y red/blue");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            turret.stop();
            return;
        }

        loopTimer.reset();
        try {
            while (opModeIsActive()) {
                double dtSeconds = loopTimer.seconds();
                loopTimer.reset();

                updateTuningControls();
                updateDrive(robot);

                boolean zeroPressed = gamepad1.left_stick_button
                        && gamepad1.right_stick_button;
                if (zeroPressed && !lastZeroButton) {
                    turret.zeroAtCurrentPosition();
                }
                lastZeroButton = zeroPressed;

                boolean autoAim = gamepad1.right_trigger > 0.20 && !zeroPressed;
                turret.setAutoAimEnabled(autoAim);
                double manualTurret = gamepad1.back
                        ? (gamepad1.dpad_right ? 1.0 : 0.0)
                                - (gamepad1.dpad_left ? 1.0 : 0.0)
                        : 0.0;
                turret.setManualPower(manualTurret * MANUAL_TURRET_POWER);
                turret.update(dtSeconds);

                showTelemetry(robot, turret, autoAim);
            }
        } finally {
            robot.lf.setPower(0);
            robot.lb.setPower(0);
            robot.rf.setPower(0);
            robot.rb.setPower(0);
            turret.stop();
        }
    }

    private void updateDrive(Hardware robot) {
        double forward = -deadband(gamepad1.left_stick_y) * DRIVE_SPEED;
        double strafe = deadband(gamepad1.left_stick_x) * DRIVE_SPEED;
        double turn = deadband(gamepad1.right_stick_x) * DRIVE_SPEED;
        double denominator = Math.max(
                Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);

        robot.lf.setPower((forward + strafe + turn) / denominator);
        robot.lb.setPower((forward - strafe + turn) / denominator);
        robot.rf.setPower((forward - strafe - turn) / denominator);
        robot.rb.setPower((forward + strafe - turn) / denominator);
    }

    private void updateTuningControls() {
        if (gamepad1.dpad_up && !lastDpadUp) {
            selectedParameter = (selectedParameter - 1 + PARAMETER_NAMES.length)
                    % PARAMETER_NAMES.length;
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            selectedParameter = (selectedParameter + 1) % PARAMETER_NAMES.length;
        }

        double multiplier = gamepad1.left_bumper ? 0.1
                : gamepad1.right_bumper ? 10.0 : 1.0;
        if (!gamepad1.back && gamepad1.dpad_left && !lastDpadLeft) {
            adjustSelected(-PARAMETER_STEPS[selectedParameter] * multiplier);
        }
        if (!gamepad1.back && gamepad1.dpad_right && !lastDpadRight) {
            adjustSelected(PARAMETER_STEPS[selectedParameter] * multiplier);
        }

        if (gamepad1.a && !lastPredictionButton) {
            turretSubsystem2.PREDICTION_ENABLED = !turretSubsystem2.PREDICTION_ENABLED;
        }
        if (gamepad1.b && !lastOdometryButton) {
            turretSubsystem2.USE_ODOMETRY_PREDICTION =
                    !turretSubsystem2.USE_ODOMETRY_PREDICTION;
        }
        if (gamepad1.x && !lastLatencyButton) {
            turretSubsystem2.LATENCY_COMPENSATION_ENABLED =
                    !turretSubsystem2.LATENCY_COMPENSATION_ENABLED;
        }
        if (gamepad1.y && !lastAllianceButton) {
            turretSubsystem2.TARGET_CLUSTER_FIRST_ID =
                    turretSubsystem2.TARGET_CLUSTER_FIRST_ID == 42 ? 30 : 42;
        }

        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
        lastPredictionButton = gamepad1.a;
        lastOdometryButton = gamepad1.b;
        lastLatencyButton = gamepad1.x;
        lastAllianceButton = gamepad1.y;
    }

    private void adjustSelected(double amount) {
        switch (selectedParameter) {
            case 0:
                turretSubsystem2.POSITION_KP = nonnegative(
                        turretSubsystem2.POSITION_KP + amount);
                break;
            case 1:
                turretSubsystem2.VELOCITY_KD = nonnegative(
                        turretSubsystem2.VELOCITY_KD + amount);
                break;
            case 2:
                turretSubsystem2.POSITION_KI = nonnegative(
                        turretSubsystem2.POSITION_KI + amount);
                break;
            case 3:
                turretSubsystem2.VELOCITY_KV = nonnegative(
                        turretSubsystem2.VELOCITY_KV + amount);
                break;
            case 4:
                turretSubsystem2.STICTION_POWER = clip(
                        turretSubsystem2.STICTION_POWER + amount, 0, 0.30);
                break;
            case 5:
                turretSubsystem2.MAX_INTEGRAL_POWER = clip(
                        turretSubsystem2.MAX_INTEGRAL_POWER + amount, 0, 0.30);
                break;
            case 6:
                turretSubsystem2.VISION_CORRECTION_GAIN = clip(
                        turretSubsystem2.VISION_CORRECTION_GAIN + amount, 0.01, 1.0);
                break;
            case 7:
                turretSubsystem2.AIM_TOLERANCE_DEG = clip(
                        turretSubsystem2.AIM_TOLERANCE_DEG + amount, 0.05, 5.0);
                break;
            case 8:
                turretSubsystem2.MAX_TURRET_POWER = clip(
                        turretSubsystem2.MAX_TURRET_POWER + amount, 0.05, 1.0);
                break;
            case 9:
                turretSubsystem2.ODOMETRY_RATE_BLEND = clip(
                        turretSubsystem2.ODOMETRY_RATE_BLEND + amount, 0, 1.0);
                break;
            case 10:
                turretSubsystem2.FIXED_FLIGHT_TIME_S = nonnegative(
                        turretSubsystem2.FIXED_FLIGHT_TIME_S + amount);
                break;
            case 11:
                turretSubsystem2.TOF_PER_INCH_S = nonnegative(
                        turretSubsystem2.TOF_PER_INCH_S + amount);
                break;
            case 12:
                turretSubsystem2.FIXED_SHOT_DELAY_S = nonnegative(
                        turretSubsystem2.FIXED_SHOT_DELAY_S + amount);
                break;
            default:
                break;
        }
    }

    private void showTelemetry(
            Hardware robot, turretSubsystem2 turret, boolean autoAim) {
        telemetry.addData("MODE", autoAim ? "AUTO-AIM" : "MANUAL");
        telemetry.addData("SELECTED", "%s = %.6f",
                PARAMETER_NAMES[selectedParameter], getSelectedValue());
        telemetry.addData("Step", "%.6f (LB x0.1, RB x10)",
                PARAMETER_STEPS[selectedParameter]);
        telemetry.addData("Cluster", "%s IDs %d-%d",
                turretSubsystem2.TARGET_CLUSTER_FIRST_ID == 42 ? "BLUE SCORING" : "RED SCORING",
                turretSubsystem2.TARGET_CLUSTER_FIRST_ID,
                turretSubsystem2.TARGET_CLUSTER_FIRST_ID + 3);
        telemetry.addData("Features", "prediction=%s, odometry=%s, latency=%s",
                onOff(turretSubsystem2.PREDICTION_ENABLED),
                onOff(turretSubsystem2.USE_ODOMETRY_PREDICTION),
                onOff(turretSubsystem2.LATENCY_COMPENSATION_ENABLED));
        telemetry.addData("Target", "%s | tags %s | cluster geometry %s",
                turret.hasTarget() ? "VISIBLE" : "lost",
                turret.getVisibleTagIds(),
                turret.isClusterGeometryValid() ? "SOLVED" : "fallback");
        telemetry.addData("PID", "err=%+.2f tk vel=%+.1f tk/s power=%+.3f I=%+.3f",
                turret.getPositionErrorTicks(),
                turret.getFilteredMotorVelocityTicksPerSecond(),
                turret.getAppliedPower(), turret.getIntegralPower());
        telemetry.addData("Turret ticks", "raw=%d zero=%d relative=%d target=%.1f",
                robot.getTurretRawEncoderTicks(),
                robot.getTurretEncoderZeroTicks(),
                turret.getCurrentTicks(), turret.getTargetTicks());
        telemetry.addData("Turret hardware", "%s | %s",
                robot.turret.getConnectionInfo(), robot.turret.getMode());
        telemetry.addData("Vision", "tx=%+.2f err=%+.2f dist=%.1f in",
                turret.getFilteredTxDeg(), turret.getLastVisionErrorDeg(),
                turret.getDistanceIn());
        telemetry.addData("Robot velocity", "forward=%+.1f left=%+.1f turn=%+.1f deg/s (%s)",
                turret.getRobotForwardVelocityInPerSec(),
                turret.getRobotLeftVelocityInPerSec(),
                turret.getRobotHeadingRateDegPerSec(),
                turret.isOdometryRateValid() ? "valid" : "INVALID");
        telemetry.addData("Bearing rate", "vision=%+.1f odo=%+.1f used=%+.1f deg/s",
                turret.getVisionTargetRateDegPerSec(),
                turret.getOdometryTargetRateDegPerSec(),
                turret.getTargetRateDegPerSec());
        telemetry.addData("Range rate", "vision=%+.1f odo=%+.1f used=%+.1f in/s",
                turret.getVisionRangeRateInPerSec(),
                turret.getOdometryRangeRateInPerSec(),
                turret.getRangeRateInPerSec());
        telemetry.addData("Lead", "%+.2f deg, %.3f s, impact %.1f in",
                turret.getPanLeadDeg(), turret.getPredictionTimeS(),
                turret.getImpactDistanceIn());
        telemetry.addLine("Tune PID with prediction/odometry OFF first; record final values.");
        telemetry.update();
    }

    private double getSelectedValue() {
        switch (selectedParameter) {
            case 0: return turretSubsystem2.POSITION_KP;
            case 1: return turretSubsystem2.VELOCITY_KD;
            case 2: return turretSubsystem2.POSITION_KI;
            case 3: return turretSubsystem2.VELOCITY_KV;
            case 4: return turretSubsystem2.STICTION_POWER;
            case 5: return turretSubsystem2.MAX_INTEGRAL_POWER;
            case 6: return turretSubsystem2.VISION_CORRECTION_GAIN;
            case 7: return turretSubsystem2.AIM_TOLERANCE_DEG;
            case 8: return turretSubsystem2.MAX_TURRET_POWER;
            case 9: return turretSubsystem2.ODOMETRY_RATE_BLEND;
            case 10: return turretSubsystem2.FIXED_FLIGHT_TIME_S;
            case 11: return turretSubsystem2.TOF_PER_INCH_S;
            case 12: return turretSubsystem2.FIXED_SHOT_DELAY_S;
            default: return Double.NaN;
        }
    }

    private double deadband(double value) {
        return Math.abs(value) < 0.05 ? 0 : value;
    }

    private double nonnegative(double value) {
        return Math.max(0, value);
    }

    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private String onOff(boolean value) {
        return value ? "ON" : "off";
    }
}
