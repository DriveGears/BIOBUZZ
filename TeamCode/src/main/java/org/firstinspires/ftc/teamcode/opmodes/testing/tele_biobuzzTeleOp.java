package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.math.Pose;

import static com.pedropathing.api.Paths.line;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.turretSubsystem;

@TeleOp(name = "OFFICIAL USE THIS TeleOp", group = "TeleOp")
public class tele_biobuzzTeleOp extends LinearOpMode {

    private Follower follower;
    private turretSubsystem turret;
    private ElapsedTime loopTimer;

    private boolean goingToFlower = false;
    private boolean previousX = false;

    private static final double[][] FLOWERS = {
            {1.46, 46.44, 180},   // West
            {46.73, 139.72, 90},  // North
            {140.33, 94.29, 0},   // East
            {94.90, 0.43, 270}    // South
    };

    private static final double INTAKE_FORWARD_IN = 8.5;
    private static final double INTAKE_LEFT_IN = 8.5;

    private Pose flowerTarget;
    private int selectedFlower = -1;

    @Override
    public void runOpMode() {
        Hardware.resetInstance();
        Hardware robot = Hardware.getInstance(hardwareMap);

        follower = Constants.create(hardwareMap);
        turret = new turretSubsystem(robot);
        loopTimer = new ElapsedTime();

        robot.lf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lb.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rf.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rb.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Robot Initialized with Turret & Robot-Centric Drive");
        telemetry.addLine("X: nearest flower | B: cancel | Hold Left Bumper: Turret Auto-Aim");
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
        previousX = gamepad1.x;
        loopTimer.reset();

        while (opModeIsActive()) {
            double dtSeconds = loopTimer.seconds();
            loopTimer.reset();

            double forward = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            boolean intake = gamepad1.right_trigger > 0.5;
            boolean outtake = gamepad1.left_trigger > 0.5;
            boolean spit = gamepad1.right_bumper;

            if (intake) {
                robot.intake.setPower(0.8);
            } else if (spit) {
                robot.intake.setPower(-0.8);
            } else {
                robot.intake.setPower(0);
            }

            if (outtake) {
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

            // Detect a new X press.
            boolean xPressed = gamepad1.x && !previousX;
            previousX = gamepad1.x;

            boolean cancelRequested = gamepad1.b;

            if (cancelRequested) {
                goingToFlower = false;
            }

            if (xPressed && !goingToFlower && !cancelRequested) {
                goToNearestFlower();
            }

            if (!goingToFlower) {
                double denominator = Math.max(Math.abs(forward) + Math.abs(lateral) + Math.abs(turn), 1.0);
                robot.lf.setPower((forward + lateral + turn) / denominator);
                robot.lb.setPower((forward - lateral + turn) / denominator);
                robot.rf.setPower((forward - lateral - turn) / denominator);
                robot.rb.setPower((forward + lateral - turn) / denominator);
            } else {
                follower.update();
            }

            if (goingToFlower && !follower.isBusy()) {
                goingToFlower = false;
            }

            Pose robotPose = follower.pose();

            telemetry.addData("Turret Mode", autoAim ? "AUTO-AIM" : "MANUAL");
            telemetry.addData("Target Visible", turret.hasTarget() ? "YES (ID: " + turret.getSelectedTagId() + ")" : "NO");
            telemetry.addData("Aim Ready?", turret.isAimReady() ? "READY" : "ALIGNING");
            telemetry.addData("Intake", intake);
            telemetry.addData("Outtake", outtake);
            telemetry.addData("Robot X", robotPose.x());
            telemetry.addData("Robot Y", robotPose.y());
            telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));

            telemetry.update();
        }

        turret.stop();
    }

    private void goToNearestFlower() {
        Pose current = follower.pose();

        if (current == null
                || !Double.isFinite(current.x())
                || !Double.isFinite(current.y())
                || !Double.isFinite(current.heading())) {
            return;
        }

        double closestDistance = Double.MAX_VALUE;
        selectedFlower = -1;

        for (int i = 0; i < FLOWERS.length; i++) {
            double dx = current.x() - FLOWERS[i][0];
            double dy = current.y() - FLOWERS[i][1];

            double distance = Math.hypot(dx, dy);

            if (distance < closestDistance) {
                closestDistance = distance;
                selectedFlower = i;
            }
        }

        if (selectedFlower == -1) {
            return;
        }

        double flowerX = FLOWERS[selectedFlower][0];
        double flowerY = FLOWERS[selectedFlower][1];
        double flowerHeading =
                Math.toRadians(FLOWERS[selectedFlower][2]);

        double targetX = flowerX
                - INTAKE_FORWARD_IN * Math.cos(flowerHeading)
                + INTAKE_LEFT_IN * Math.sin(flowerHeading);

        double targetY = flowerY
                - INTAKE_FORWARD_IN * Math.sin(flowerHeading)
                - INTAKE_LEFT_IN * Math.cos(flowerHeading);

        flowerTarget = new Pose(targetX, targetY, flowerHeading);

        if (Math.hypot(
                targetX - current.x(),
                targetY - current.y()
        ) < 0.1) {
            return;
        }

        Pose start = new Pose(
                current.x(),
                current.y(),
                current.heading()
        );

        Path path = line(start, flowerTarget)
                .linear(start, flowerTarget);

        follower.follow(path);
        goingToFlower = true;
    }
}
