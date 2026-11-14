package org.firstinspires.ftc.teamcode.opmodes.competition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.FlightRecorder;
import org.firstinspires.ftc.teamcode.subsystems.shooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.turretSubsystem;

/** Shared implementation for the alliance-specific competition TeleOps. */
public abstract class biobuzzTeleop extends LinearOpMode {

    private FlightRecorder rec;
    private boolean lastFusionToggle = false;
    private turretSubsystem turret;
    private shooterSubsystem shooter;
    private ElapsedTime loopTimer;

    private boolean intakeToggle = false;
    private boolean outtakeToggle = false;
    private boolean lastRightTrigger = false;
    private boolean lastLeftTrigger = false;
    private boolean lastLeftBumper = false;
    private boolean autoAimEnabled = true;

    protected abstract turretSubsystem.Alliance competitionAlliance();

    @Override
    public void runOpMode() {
        Hardware.resetInstance();
        Hardware robot = Hardware.getInstance(hardwareMap);

        // The Pedro Follower was only used for a pose readout and cost a second Pinpoint I2C read every
        // loop (the turret already reads Pinpoint). Pose now comes from the Pinpoint via Hardware.
        turret = new turretSubsystem(robot);
        shooter = new shooterSubsystem(robot);
        loopTimer = new ElapsedTime();

        rec = FlightRecorder.start("teleop");
        java.util.Map<String, String> cfg = turretSubsystem.configSnapshot();
        cfg.put("opmode", getClass().getSimpleName());
        cfg.put("tune_version", turretSubsystem.TUNE_VERSION);
        rec.writeConfig(cfg);
        rec.event("--- ODOMETRY INIT ---\n" + robot.getOdometryReport());
        turret.setRecorder(rec);
        robot.enableManualBulkReads();     // one hub read per loop; beginLoop() below refreshes it
        telemetry.setMsTransmissionInterval(100);

        turret.setAlliance(competitionAlliance());
        turret.setAutoAimEnabled(true);
        shooter.setTargetRpm(shooterSubsystem.COMPETITION_RPM);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Robot Initialized with Turret & Robot-Centric Drive");
            telemetry.addData("Fixed Alliance", turret.getAlliance());
            telemetry.addLine("Auto-aim will start ENABLED");
            addControlsTelemetry();
            telemetry.update();
            idle();
        }

        if (isStopRequested()) {
            turret.stop();
            shooter.stop();
            robot.intake.setPower(0);
            rec.close();
            return;
        }

        rec.event("OPMODE_START tune=" + turretSubsystem.TUNE_VERSION + " alliance=" + competitionAlliance());
        loopTimer.reset();
        lastRightTrigger = gamepad1.right_trigger > 0.5;
        lastLeftTrigger = gamepad1.left_trigger > 0.5;
        lastLeftBumper = gamepad1.left_bumper;

        try {
        while (opModeIsActive()) {
            robot.beginLoop();
            double dtSeconds = loopTimer.seconds();
            loopTimer.reset();

            double forward = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            boolean currentRightTrigger = gamepad1.right_trigger > 0.5;
            if (currentRightTrigger && !lastRightTrigger) {
                intakeToggle = !intakeToggle;
            }
            lastRightTrigger = currentRightTrigger;

            boolean currentLeftTrigger = gamepad1.left_trigger > 0.5;
            if (currentLeftTrigger && !lastLeftTrigger) {
                outtakeToggle = !outtakeToggle;
            }
            lastLeftTrigger = currentLeftTrigger;

            boolean spit = gamepad1.right_bumper;

            if (spit) {
                robot.intake.setPower(-1);
            } else {
                robot.intake.setPower(intakeToggle ? 1 : 0.0);
            }

            shooter.setShooterEnabled(outtakeToggle);
            shooter.update(dtSeconds);
            boolean transferFeeding = gamepad1.a;
            if (transferFeeding) {
                robot.transfer.setPower(shooterSubsystem.TRANSFER_POWER);
            } else {
                robot.transfer.setPower(0);
            }

            // Auto-aim starts enabled. Tap LB to switch to/from manual aiming.
            boolean currentLeftBumper = gamepad1.left_bumper;
            if (currentLeftBumper && !lastLeftBumper) {
                autoAimEnabled = !autoAimEnabled;
            }
            lastLeftBumper = currentLeftBumper;
            turret.setAutoAimEnabled(autoAimEnabled);

            // D-pad up toggles odometry fusion (compare against vision-only aiming).
            if (gamepad1.dpad_up && !lastFusionToggle) {
                turretSubsystem.ODOMETRY_FUSION_ENABLED = !turretSubsystem.ODOMETRY_FUSION_ENABLED;
                rec.event("FUSION_TOGGLE enabled=" + turretSubsystem.ODOMETRY_FUSION_ENABLED);
            }
            lastFusionToggle = gamepad1.dpad_up;

            double manualTurretCommand = (gamepad1.dpad_right ? 1.0 : 0.0)
                    - (gamepad1.dpad_left ? 1.0 : 0.0);
            turret.setManualPower(manualTurretCommand * 0.25);
            turret.update(dtSeconds);

            // Auto-aim moves only the turret. Chassis rotation is always the
            // driver's right-stick command.
            double driveTurn = turn;

            double denominator = Math.max(
                    Math.abs(forward) + Math.abs(lateral) + Math.abs(driveTurn), 1.0);
            robot.lf.setPower((forward + lateral + driveTurn) / denominator);
            robot.lb.setPower((forward - lateral + driveTurn) / denominator);
            robot.rf.setPower((forward - lateral - driveTurn) / denominator);
            robot.rb.setPower((forward + lateral - driveTurn) / denominator);

            // ---- flight recorder row ----
            turret.logTo(rec);
            rec.put("op_dt_ms", dtSeconds * 1000.0);
            rec.put("in_forward", forward);
            rec.put("in_lateral", lateral);
            rec.put("in_turn", turn);
            rec.put("in_autoaim", autoAimEnabled);
            rec.put("in_dpad_manual", manualTurretCommand);
            rec.put("in_intake_on", intakeToggle);
            rec.put("in_outtake_on", outtakeToggle);
            rec.put("shoot_target_rpm", shooter.getTargetRpm());
            rec.put("shoot_actual_rpm", shooter.getActualRpm());
            rec.put("shoot_at_speed", shooter.isAtSpeed());
            rec.endRow();

            telemetry.addData("Turret Mode", autoAimEnabled ? "AUTO-AIM" : "MANUAL");
            telemetry.addData("Target Visible", turret.hasTarget() ? "YES (ID: " + turret.getSelectedTagId() + ")" : "NO");
            telemetry.addData("Alliance / CELL", "%s / %s",
                    turret.getAlliance(), turret.getSelectedClusterName());
            telemetry.addData("CELL Roll", Double.isFinite(turret.getSelectedClusterRollDeg())
                    ? String.format("%.1f deg (UP)", turret.getSelectedClusterRollDeg())
                    : "no scorable CELL");
            telemetry.addData("Prediction Lead", "%.2f deg", turret.getPanLeadDeg());
            telemetry.addData("Chassis Aim Assist", "DISABLED");
            telemetry.addData("Aim Ready?", turret.isAimReady() ? "READY" : "ALIGNING");
            telemetry.addData("Intake Toggle", intakeToggle ? "ON" : "OFF");
            telemetry.addData("Outtake Toggle", outtakeToggle ? "ON" : "OFF");
            telemetry.addData("Shooter RPM", "target=%.0f actual=%.0f %s",
                    shooter.getTargetRpm(), shooter.getActualRpm(),
                    !outtakeToggle ? "OFF"
                            : shooter.isAtSpeed() ? "READY" : "SPINNING UP");
            telemetry.addData("Transfer", transferFeeding ? "FEEDING (A)" : "OFF");
            telemetry.addData("Odo pose", "x=%.1f y=%.1f hdg=%.1f",
                    robot.odo.getPosX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH),
                    robot.odo.getPosY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH),
                    Math.toDegrees(robot.odo.getHeading(
                            org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS)));
            telemetry.addData("FUSION (D-pad up)", turretSubsystem.ODOMETRY_FUSION_ENABLED ? "ON" : "OFF (vision only)");
            telemetry.addData("Ready after stop (ms)", "%.0f", turret.getReadyAfterStopMs());
            telemetry.addData("Log", rec.getStatus());
            addControlsTelemetry();

            telemetry.update();
        }
        } catch (Throwable t) {
            rec.exception("biobuzzTeleop loop", t);
            throw t;
        } finally {
            robot.lf.setPower(0);
            robot.lb.setPower(0);
            robot.rf.setPower(0);
            robot.rb.setPower(0);
            turret.stop();
            shooter.stop();
            robot.intake.setPower(0);
            rec.event("OPMODE_END");
            rec.close();
        }
    }

    private void addControlsTelemetry() {
        telemetry.addLine();
        telemetry.addLine("=== GAMEPAD 1 CONTROLS ===");
        telemetry.addData("Alliance", competitionAlliance());
        telemetry.addLine("Left Stick: Drive/strafe | Right Stick X: Turn");
        telemetry.addLine("RT (tap): Toggle intake | RB (hold): Reverse intake");
        telemetry.addLine("LT (tap): Toggle flywheel | A (hold): Transfer");
        telemetry.addLine("LB (tap): Toggle auto/manual (starts AUTO)");
        telemetry.addLine("D-pad L/R: Move turret while in manual mode");
        telemetry.addLine("Auto-aim moves turret only; right stick owns chassis turn");
    }
}
