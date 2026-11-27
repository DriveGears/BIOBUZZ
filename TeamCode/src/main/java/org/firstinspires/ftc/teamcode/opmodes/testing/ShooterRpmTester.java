package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.FlightRecorder;
import org.firstinspires.ftc.teamcode.subsystems.shooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.turretSubsystem;

@TeleOp(name = "Shooter RPM Tester", group = "Testing")
public class ShooterRpmTester extends LinearOpMode {

    private static final double INTAKE_POWER = 1.0;
    private static final double TICKS_PER_REV = 28.0;
    private static final double KP = 0.0020;
    private static final double KF = 0.000435;
    private static final double SHOOTER_RPM_STEP = 25.0;
    private static final double TRANSFER_RPM_STEP = 100.0;
    private static final double DASHBOARD_PERIOD_SECONDS = 0.05;

    @Override
    public void runOpMode() {
        Hardware.resetInstance();
        Hardware robot = Hardware.getInstance(hardwareMap);
        shooterSubsystem shooter = new shooterSubsystem(robot);
        turretSubsystem turret = new turretSubsystem(robot);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        FlightRecorder recorder = FlightRecorder.start("shooter_rpm_test");
        java.util.Map<String, String> config = turretSubsystem.configSnapshot();
        config.put("opmode", "ShooterRpmTester");
        config.put("shooter_rpm_step", String.valueOf(SHOOTER_RPM_STEP));
        config.put("transfer_rpm_step", String.valueOf(TRANSFER_RPM_STEP));
        recorder.writeConfig(config);
        turret.setRecorder(recorder);
        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime dashboardTimer = new ElapsedTime();

        double targetRpm = shooterSubsystem.COMPETITION_RPM;
        double targetTransferRpm = 1500.0;
        shooter.setTargetRpm(targetRpm);

        boolean intakeOn = false;
        boolean shooterOn = false;

        boolean lastRightTrigger = false;
        boolean lastLeftTrigger = false;
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;
        boolean lastDpadLeft = false;
        boolean lastDpadRight = false;

        telemetry.addLine("Shooter & Transfer RPM Tester Ready");
        telemetry.addLine("LT (tap): Toggle Shooter | RT (tap): Toggle Intake");
        telemetry.addLine("A (hold): Transfer | D-pad Up/Down: Shooter +/-25 RPM");
        telemetry.addLine("D-pad L/R: Transfer +/-100 RPM | X: RED tags | Y: BLUE tags");
        telemetry.addLine("Turret motor is disabled in this test; Limelight is distance-only.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            turret.stop();
            shooter.stop();
            recorder.close();
            return;
        }

        loopTimer.reset();
        dashboardTimer.reset();

        try {
            while (opModeIsActive()) {
                robot.beginLoop();
                double dtSeconds = loopTimer.seconds();
                loopTimer.reset();

                // Distance sensing only. Auto-aim stays off and manual power is zero,
                // so this tester can never command turret motion.
                if (gamepad1.x) turret.setAlliance(turretSubsystem.Alliance.RED);
                if (gamepad1.y) turret.setAlliance(turretSubsystem.Alliance.BLUE);
                turret.update(dtSeconds, false, 0.0);

                // 1. Intake Toggle (RT)
                boolean currentRightTrigger = gamepad1.right_trigger > 0.5;
                if (currentRightTrigger && !lastRightTrigger) {
                    intakeOn = !intakeOn;
                }
                lastRightTrigger = currentRightTrigger;

                if (gamepad1.right_bumper) {
                    robot.intake.setPower(-INTAKE_POWER);
                } else {
                    robot.intake.setPower(intakeOn ? INTAKE_POWER : 0.0);
                }

                // 2. Shooter Toggle (LT)
                boolean currentLeftTrigger = gamepad1.left_trigger > 0.5;
                if (currentLeftTrigger && !lastLeftTrigger) {
                    shooterOn = !shooterOn;
                }
                lastLeftTrigger = currentLeftTrigger;

                // 3. Shooter RPM Adjustments (D-pad Up/Down +/- 25 RPM)
                boolean currentDpadUp = gamepad1.dpad_up;
                if (currentDpadUp && !lastDpadUp) {
                    targetRpm += SHOOTER_RPM_STEP;
                    shooter.setTargetRpm(targetRpm);
                }
                lastDpadUp = currentDpadUp;

                boolean currentDpadDown = gamepad1.dpad_down;
                if (currentDpadDown && !lastDpadDown) {
                    targetRpm = Math.max(0, targetRpm - SHOOTER_RPM_STEP);
                    shooter.setTargetRpm(targetRpm);
                }
                lastDpadDown = currentDpadDown;

                shooter.setShooterEnabled(shooterOn);
                shooter.update(dtSeconds);

                // 4. Transfer RPM Adjustments (D-pad Left/Right +/- 100 RPM)
                boolean currentDpadLeft = gamepad1.dpad_left;
                if (currentDpadLeft && !lastDpadLeft) {
                    targetTransferRpm = Math.max(0, targetTransferRpm - TRANSFER_RPM_STEP);
                }
                lastDpadLeft = currentDpadLeft;

                boolean currentDpadRight = gamepad1.dpad_right;
                if (currentDpadRight && !lastDpadRight) {
                    targetTransferRpm += TRANSFER_RPM_STEP;
                }
                lastDpadRight = currentDpadRight;

                // 5. Transfer Feed (A hold) with RPM PIDF
                boolean transferFeeding = gamepad1.a;
                double transferPower = 0.0;
                double actualTransferTps = robot.transfer.getVelocity();
                double actualTransferRpm = (actualTransferTps * 60.0) / TICKS_PER_REV;
                if (transferFeeding) {
                    double targetTransferTps = targetTransferRpm * TICKS_PER_REV / 60.0;
                    double errorTps = targetTransferTps - actualTransferTps;

                    transferPower = (KP * errorTps) + (KF * targetTransferTps);
                    transferPower = Math.max(-1.0, Math.min(1.0, transferPower));
                }
                robot.transfer.setPower(transferFeeding ? transferPower : 0.0);

                // Save the same numeric channels used for Dashboard graphs. The files
                // remain on the Control Hub even when Dashboard has no download button.
                turret.logTo(recorder);
                shooter.logTo(recorder);
                recorder.put("test_selected_rpm", targetRpm);
                recorder.put("test_distance_used_in",
                        turret.hasDistance() ? turret.getShooterDistanceIn() : Double.NaN);
                recorder.put("test_transfer_target_rpm", targetTransferRpm);
                recorder.put("test_transfer_actual_rpm", actualTransferRpm);
                recorder.put("test_transfer_power", transferPower);
                recorder.put("test_transfer_feeding", transferFeeding);
                recorder.endRow();

                // Telemetry
                telemetry.addData("Shooter Status", shooterOn ? (shooter.isAtSpeed() ? "READY" : "SPINNING UP") : "OFF");
                telemetry.addData("Shooter Target RPM", "%.0f", targetRpm);
                telemetry.addData("Shooter Actual RPM", "%.0f", shooter.getActualRpm());
                telemetry.addData("Selected Alliance", turret.getAlliance());
                telemetry.addData("Visible Tags", "%s", turret.getVisibleTagIds());
                telemetry.addData("TARGET DISTANCE", formatUsedDistance(turret));
                telemetry.addData("Raw Vision Distance", formatDistance(
                        turret.getRawDistanceIn()));
                telemetry.addData("Tracked Distance", formatDistance(
                        turret.hasDistance() ? turret.getDistanceIn() : Double.NaN));
                telemetry.addData("Fused Distance", turret.hasFusedDistance()
                        ? formatDistance(turret.getFusedDistanceIn())
                        : "N/A (vision distance is still usable)");
                telemetry.addData("Distance Status", distanceStatus(turret));
                telemetry.addData("Range Protection", "shotFreeze=%s disagreement=%+.1f in",
                        turret.isDistanceFrozenAfterShot(),
                        turret.getRangeDisagreementIn());
                telemetry.addLine();
                telemetry.addData("Transfer Status", transferFeeding ? "FEEDING" : "OFF");
                telemetry.addData("Transfer Target RPM", "%.0f", targetTransferRpm);
                telemetry.addData("Transfer Actual RPM", "%.0f", actualTransferRpm);
                telemetry.addData("Transfer Power", "%.2f", transferPower);
                telemetry.update();

                if (dashboardTimer.seconds() >= DASHBOARD_PERIOD_SECONDS) {
                    dashboardTimer.reset();
                    TelemetryPacket packet = new TelemetryPacket();
                    double actualShooterRpm = shooter.getActualRpm();
                    packet.put("shooter.target_rpm", shooter.getTargetRpm());
                    packet.put("shooter.commanded_rpm", shooter.getCommandedRpm());
                    packet.put("shooter.actual_rpm", actualShooterRpm);
                    packet.put("shooter.error_rpm", shooter.getTargetRpm() - actualShooterRpm);
                    packet.put("shooter.power", shooter.getAppliedPower());
                    packet.put("shooter.p_term", shooter.getPTerm());
                    packet.put("shooter.i_term", shooter.getITerm());
                    packet.put("shooter.d_term", shooter.getDTerm());
                    packet.put("shooter.f_term", shooter.getFTerm());
                    packet.put("shooter.recovery_boost", shooter.getRecoveryBoostPower());
                    packet.put("shooter.ready", shooter.isAtSpeed() ? 1.0 : 0.0);

                    packet.put("transfer.target_rpm", targetTransferRpm);
                    packet.put("transfer.actual_rpm", actualTransferRpm);
                    packet.put("transfer.power", transferPower);
                    packet.put("transfer.feeding", transferFeeding ? 1.0 : 0.0);

                    putIfFinite(packet, "distance.used_in",
                            turret.hasDistance() ? turret.getShooterDistanceIn() : Double.NaN);
                    putIfFinite(packet, "distance.raw_vision_in", turret.getRawDistanceIn());
                    putIfFinite(packet, "distance.tracked_in",
                            turret.hasDistance() ? turret.getDistanceIn() : Double.NaN);
                    putIfFinite(packet, "distance.fused_in",
                            turret.hasFusedDistance() ? turret.getFusedDistanceIn() : Double.NaN);
                    putIfFinite(packet, "distance.vision_minus_fused_in",
                            turret.getRangeDisagreementIn());
                    packet.put("distance.shot_freeze",
                            turret.isDistanceFrozenAfterShot() ? 1.0 : 0.0);
                    packet.put("vision.target_visible", turret.hasTarget() ? 1.0 : 0.0);
                    packet.put("vision.tx_deg", turret.getFilteredTxDeg());
                    packet.put("vision.selected_tag_id", turret.getSelectedTagId());
                    packet.put("vision.cluster_first_id", turret.getSelectedClusterFirstId());

                    packet.put("turret.current_ticks", turret.getCurrentTicks());
                    packet.put("turret.power", turret.getAppliedPower());
                    dashboard.sendTelemetryPacket(packet);
                }
            }
        } finally {
            turret.stop();
            shooter.stop();
            robot.intake.setPower(0);
            robot.transfer.setPower(0);
            recorder.close();
        }
    }

    private void putIfFinite(TelemetryPacket packet, String key, double value) {
        if (Double.isFinite(value)) packet.put(key, value);
    }

    private String formatUsedDistance(turretSubsystem turret) {
        double distance = turret.getShooterDistanceIn();
        return turret.hasDistance() && Double.isFinite(distance) && distance > 0
                ? formatDistance(distance) : "NO VALID DISTANCE";
    }

    private String formatDistance(double distance) {
        return Double.isFinite(distance) && distance > 0
                ? String.format(java.util.Locale.US, "%.1f in", distance) : "N/A";
    }

    private String distanceStatus(turretSubsystem turret) {
        if (turret.hasDistance() && Double.isFinite(turret.getShooterDistanceIn())) {
            if (turret.isDistanceFrozenAfterShot()) return "SHOT HOLD: LAST TRUSTED RANGE";
            if (turret.hasFusedDistance()) {
                return turret.hasTarget()
                        ? "FUSED RANGE (VISION + ODOMETRY)"
                        : "ODOMETRY-TRACKED RANGE";
            }
            return "FILTERED VISION RANGE";
        }
        if (turret.hasTarget()) {
            return "TAG FOUND, BUT CAMERA HEIGHT/ANGLE DID NOT PRODUCE A RANGE";
        }
        String ids = turret.getVisibleTagIds();
        if (ids != null && !ids.equals("none") && !ids.equals("none allowed")) {
            return "TAG SEEN, BUT CELL ORIENTATION WAS REJECTED";
        }
        return "NO ALLOWED TAG: aim camera at HIVE; X=RED, Y=BLUE";
    }
}
