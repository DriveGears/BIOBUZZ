package org.firstinspires.ftc.teamcode.opmodes.competition;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.shooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.turretSubsystem;

/** Shared implementation for the alliance-specific competition TeleOps. */
public abstract class biobuzzTeleop extends LinearOpMode {

    private Follower follower;
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

        follower = Constants.create(hardwareMap);
        turret = new turretSubsystem(robot);
        shooter = new shooterSubsystem(robot);
        loopTimer = new ElapsedTime();

        follower.setPose(new Pose(
                56.0,
                8.0,
                Math.toRadians(0)
        ));
        turret.setAlliance(competitionAlliance());
        turret.setAutoAimEnabled(true);
        shooter.setTargetRpm(shooterSubsystem.COMPETITION_RPM);

        while (!isStarted() && !isStopRequested()) {
            follower.update();
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
            return;
        }

        loopTimer.reset();
        lastRightTrigger = gamepad1.right_trigger > 0.5;
        lastLeftTrigger = gamepad1.left_trigger > 0.5;
        lastLeftBumper = gamepad1.left_bumper;

        while (opModeIsActive()) {
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
                robot.intake.setPower(-0.7);
            } else {
                robot.intake.setPower(intakeToggle ? 0.7 : 0.0);
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

            follower.update();
            Pose robotPose = follower.pose();

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
            telemetry.addData("Robot X", robotPose.x());
            telemetry.addData("Robot Y", robotPose.y());
            telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
            addControlsTelemetry();

            telemetry.update();
        }

        follower.stop();
        turret.stop();
        shooter.stop();
        robot.intake.setPower(0);
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
