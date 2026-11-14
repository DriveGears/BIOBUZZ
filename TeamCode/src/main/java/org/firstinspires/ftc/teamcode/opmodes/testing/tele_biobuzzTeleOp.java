package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.FlightRecorder;
import org.firstinspires.ftc.teamcode.subsystems.turretSubsystem;

import java.util.Map;

@TeleOp(name = "OFFICIAL USE THIS TeleOp", group = "TeleOp")
public class tele_biobuzzTeleOp extends LinearOpMode {

    private boolean intakeToggle = false;
    private boolean outtakeToggle = false;
    private boolean lastRightTrigger = false;
    private boolean lastLeftTrigger = false;
    private boolean lastFusionToggle = false;

    @Override
    public void runOpMode() {
        Hardware.resetInstance();
        Hardware robot = Hardware.getInstance(hardwareMap);

        // The Pedro Follower used to be created and updated here, but it was only
        // used for a pose readout and cost a second Pinpoint I2C read every loop
        // (the turret already reads Pinpoint). Pose now comes from the turret's odo.
        turretSubsystem turret = new turretSubsystem(robot);
        ElapsedTime loopTimer = new ElapsedTime();

        FlightRecorder rec = FlightRecorder.start("teleop");
        Map<String, String> cfg = turretSubsystem.configSnapshot();
        cfg.put("opmode", "tele_biobuzzTeleOp");
        cfg.put("tune_version", turretSubsystem.TUNE_VERSION);
        rec.writeConfig(cfg);
        turret.setRecorder(rec);
        rec.event("--- ODOMETRY INIT ---\n" + robot.getOdometryReport());

        // One hub read per loop instead of one per getter call.
        robot.enableManualBulkReads();
        telemetry.setMsTransmissionInterval(100);

        for (String line : robot.getOdometryReport().split("\n")) {
            if (line.startsWith("ODO_CONFIGURED") || line.startsWith("ODO_WARNING")
                    || line.startsWith("ODO_LEAN") || line.startsWith("ODO_FIRMWARE")) telemetry.addLine(line);
        }
        telemetry.addLine("Robot Initialized with Turret & Robot-Centric Drive");
        telemetry.addLine("Right Trigger: Toggle Intake | Left Trigger: Toggle Outtake & Transfer");
        telemetry.addLine("Hold Left Bumper: Turret Auto-Aim | D-pad: Manual Turret");
        telemetry.addLine(rec.getStatus());
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            turret.stop();
            rec.close();
            return;
        }

        rec.event("OPMODE_START tune=" + turretSubsystem.TUNE_VERSION);
        loopTimer.reset();

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
                    robot.intake.setPower(-0.8);
                } else {
                    robot.intake.setPower(intakeToggle ? 0.8 : 0.0);
                }

                if (outtakeToggle) {
                    robot.outtake.setPower(0.8);
                    robot.transfer.setPower(0.8);
                } else {
                    robot.outtake.setPower(0);
                    robot.transfer.setPower(0);
                }

                // D-pad up toggles odometry fusion (compare against vision-only aiming).
                if (gamepad1.dpad_up && !lastFusionToggle) {
                    turretSubsystem.ODOMETRY_FUSION_ENABLED = !turretSubsystem.ODOMETRY_FUSION_ENABLED;
                    rec.event("FUSION_TOGGLE enabled=" + turretSubsystem.ODOMETRY_FUSION_ENABLED);
                }
                lastFusionToggle = gamepad1.dpad_up;

                // Turret Auto-Aim (Hold Left Bumper for Auto-Aim, D-Pad L/R for manual)
                boolean autoAim = gamepad1.left_bumper;
                turret.setAutoAimEnabled(autoAim);

                double manualTurretCommand = (gamepad1.dpad_right ? 1.0 : 0.0)
                        - (gamepad1.dpad_left ? 1.0 : 0.0);
                turret.setManualPower(manualTurretCommand * 0.25);
                turret.update(dtSeconds);

                double denominator = Math.max(Math.abs(forward) + Math.abs(lateral) + Math.abs(turn), 1.0);
                double lfP = (forward + lateral + turn) / denominator;
                double lbP = (forward - lateral + turn) / denominator;
                double rfP = (forward - lateral - turn) / denominator;
                double rbP = (forward + lateral - turn) / denominator;
                robot.lf.setPower(lfP);
                robot.lb.setPower(lbP);
                robot.rf.setPower(rfP);
                robot.rb.setPower(rbP);

                // ---- flight recorder row ----
                turret.logTo(rec);
                rec.put("op_dt_ms", dtSeconds * 1000.0);
                rec.put("in_forward", forward);
                rec.put("in_lateral", lateral);
                rec.put("in_turn", turn);
                rec.put("in_autoaim", autoAim);
                rec.put("in_dpad_manual", manualTurretCommand);
                rec.put("in_intake_on", intakeToggle);
                rec.put("in_outtake_on", outtakeToggle);
                rec.put("drive_lf", lfP);
                rec.put("drive_lb", lbP);
                rec.put("drive_rf", rfP);
                rec.put("drive_rb", rbP);
                rec.put("outtake_vel_tps", robot.outtake.getVelocity());
                rec.endRow();

                telemetry.addData("Turret Mode", autoAim ? "AUTO-AIM" : "MANUAL");
                telemetry.addData("Target Visible", turret.hasTarget()
                        ? "YES (ID: " + turret.getSelectedTagId() + ")" : "NO");
                telemetry.addData("Aim Ready?", turret.isAimReady() ? "READY" : "ALIGNING");
                telemetry.addData("Ready after stop (ms)", "%.0f", turret.getReadyAfterStopMs());
                telemetry.addData("Loop", "%.1f ms (turret %.0f us)", dtSeconds * 1000.0,
                        turret.getUpdateMicros());
                telemetry.addData("Intake Toggle", intakeToggle ? "ON" : "OFF");
                telemetry.addData("Outtake Toggle", outtakeToggle ? "ON" : "OFF");
                telemetry.addData("Odo pose", "x=%.1f y=%.1f hdg=%.1f",
                        robot.odo.getPosX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH),
                        robot.odo.getPosY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH),
                        Math.toDegrees(robot.odo.getHeading(
                                org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS)));
                telemetry.addData("FUSION (D-pad up)", turretSubsystem.ODOMETRY_FUSION_ENABLED ? "ON" : "OFF (vision only)");
                telemetry.addData("Log", rec.getStatus());
                telemetry.update();
            }
        } catch (Throwable t) {
            rec.exception("tele_biobuzzTeleOp loop", t);
            throw t;
        } finally {
            robot.lf.setPower(0);
            robot.lb.setPower(0);
            robot.rf.setPower(0);
            robot.rb.setPower(0);
            turret.stop();
            rec.event("OPMODE_END");
            rec.close();
        }
    }
}
