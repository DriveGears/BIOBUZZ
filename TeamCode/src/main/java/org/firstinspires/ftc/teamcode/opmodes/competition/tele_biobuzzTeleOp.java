package org.firstinspires.ftc.teamcode.opmodes.competition;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.turretSubsystem;

@TeleOp(name = "OFFICIAL USE THIS TeleOp", group = "TeleOp")
public class tele_biobuzzTeleOp extends LinearOpMode {

    private Follower follower;
    private turretSubsystem turret;
    private ElapsedTime loopTimer;

    private boolean intakeToggle = false;
    private boolean outtakeToggle = false;
    private boolean lastRightTrigger = false;
    private boolean lastLeftTrigger = false;

    @Override
    public void runOpMode() {
        Hardware.resetInstance();
        Hardware robot = Hardware.getInstance(hardwareMap);

        follower = Constants.create(hardwareMap);
        turret = new turretSubsystem(robot);
        loopTimer = new ElapsedTime();

        telemetry.addLine("Robot Initialized with Turret & Robot-Centric Drive");
        telemetry.addLine("Right Trigger: Toggle Intake | Left Trigger: Toggle Outtake & Transfer");
        telemetry.addLine("Hold Left Bumper: Turret Auto-Aim | D-pad: Manual Turret");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            turret.stop();
            return;
        }

        follower.setPose(new Pose(
                56.0,
                8.0,
                Math.toRadians(0)
        ));

        follower.update();
        loopTimer.reset();

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

            // Turret Auto-Aim (Hold Left Bumper for Auto-Aim, D-Pad L/R for manual)
            boolean autoAim = gamepad1.left_bumper;
            turret.setAutoAimEnabled(autoAim);

            double manualTurretCommand = (gamepad1.dpad_right ? 1.0 : 0.0)
                    - (gamepad1.dpad_left ? 1.0 : 0.0);
            turret.setManualPower(manualTurretCommand * 0.25);
            turret.update(dtSeconds);

            double denominator = Math.max(Math.abs(forward) + Math.abs(lateral) + Math.abs(turn), 1.0);
            robot.lf.setPower((forward + lateral + turn) / denominator);
            robot.lb.setPower((forward - lateral + turn) / denominator);
            robot.rf.setPower((forward - lateral - turn) / denominator);
            robot.rb.setPower((forward + lateral - turn) / denominator);

            follower.update();

            Pose robotPose = follower.pose();

            telemetry.addData("Turret Mode", autoAim ? "AUTO-AIM" : "MANUAL");
            telemetry.addData("Target Visible", turret.hasTarget() ? "YES (ID: " + turret.getSelectedTagId() + ")" : "NO");
            telemetry.addData("Aim Ready?", turret.isAimReady() ? "READY" : "ALIGNING");
            telemetry.addData("Intake Toggle", intakeToggle ? "ON" : "OFF");
            telemetry.addData("Outtake Toggle", outtakeToggle ? "ON" : "OFF");
            telemetry.addData("Robot X", robotPose.x());
            telemetry.addData("Robot Y", robotPose.y());
            telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));

            telemetry.update();
        }

        turret.stop();
    }
}
