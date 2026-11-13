package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.pedropathing.follower.Follower;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.paths.Path;
import com.pedropathing.math.Pose;

import static com.pedropathing.api.Paths.line;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;

/*
 * assumes 144inx144in field size
 *
 *  ---- CONTROLS ----
 * Intake                       right trigger (change to always)
 * Outtake and transfer         left trigger
 * Reverse intake               right bumper
 * Robot centric driving
 * Go to nearest flower         x
 * Cancel go to flower          a
 *
 * !!!! Change starting position by setting START_POSITION to 0-3 !!!!
 */

@TeleOp(name = "Basic Mecanum TeleOp", group = "TeleOp")
public class biobuzzTeleOp extends LinearOpMode {

    private Follower follower;

    private boolean goingToFlower = false;
    private boolean previousX = false;

    // Estimated flower centres from the simulator.
    // X/Y in inches, heading in degrees.
    private static final double[][] FLOWERS = {
            {0, 48, 180},   // West
            {48, 144, 90},  // North
            {144, 96, 0},   // East
            {96, 0, 270}    // South
    };

    private static final Pose[] START_POSES = {
            new Pose(60, 8, Math.toRadians(90)),         // Bottom Left
            new Pose(84, 8, Math.toRadians(90)),         // Bottom Right
            new Pose(60, 136, Math.toRadians(270)),     // Top Left
            new Pose(84, 136, Math.toRadians(270))      // Top Right
    };
    // 0 = Bottom Left
    // 1 = Bottom Right
    // 2 = Top Left
    // 3 = Top Right
    private static final int START_POSITION = 0;

    private static final String[] FLOWER_NAMES = {
            "West", "North", "East", "South"
    };

    // Measure from the robot's tracked position to the intake.
    // Positive side offset means LEFT.
    private static final double INTAKE_FORWARD_IN = 8.5;
    private static final double INTAKE_LEFT_IN = 8.5;
    private Pose flowerTarget;
    private int selectedFlower = -1;

    @Override
    public void runOpMode() {
        Hardware.resetInstance();
        Hardware robot = Hardware.getInstance(hardwareMap);

        follower = Constants.create(hardwareMap);

        robot.lf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lb.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rf.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rb.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Place the robot at the selected starting position before pressing INIT.
        follower.setPose(START_POSES[START_POSITION]);

        // Update localization while waiting for START.
        while (!isStarted() && !isStopRequested()) {
            follower.manual(0, 0, 0);
            follower.update();

            Pose current = follower.pose();

            telemetry.addLine("Robot initialized — keep stationary");
            telemetry.addData("Starting position", START_POSITION);

            if (current != null) {
                telemetry.addData("Robot X", current.x());
                telemetry.addData("Robot Y", current.y());
                telemetry.addData(
                        "Robot Heading",
                        Math.toDegrees(current.heading())
                );
            }

            telemetry.update();
            idle();
        }

        if (isStopRequested()) return;

        previousX = gamepad1.x;

        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            boolean intake = gamepad1.right_trigger > 0.5;
            boolean outtake = gamepad1.left_trigger > 0.5;
            boolean spit = gamepad1.right_bumper;

            boolean xPressed = gamepad1.x && !previousX;
            previousX = gamepad1.x;

            boolean cancelRequested = gamepad1.a;

            if (intake) {
                robot.intake.setPower(0.85);
            } else if (spit) {
                robot.intake.setPower(-0.85);
            } else {
                robot.intake.setPower(0);
            }

            if (outtake) {
                robot.outtake.setPower(0.9);
                robot.transfer.setPower(0.9);
            } else {
                robot.outtake.setPower(0);
                robot.transfer.setPower(0);
            }

            if (cancelRequested) {
                goingToFlower = false;
            }

            if (xPressed && !goingToFlower && !cancelRequested) {
                goToNearestFlower();
            }

////          // field centric
//            if (!goingToFlower) {
//                DrivePowers powers = ManualDrive.fieldCentric(
//                        forward,
//                        lateral,
//                        turn,
//                        follower.pose().heading()
//                );
//
//                follower.manual(powers);
//            }

            // robot centric
            if (!goingToFlower) {
                follower.manual(forward, lateral, turn);
            }

            follower.update();

            if (goingToFlower && !follower.isBusy()) {
                goingToFlower = false;
            }

            Pose robotPose = follower.pose();

            telemetry.addData("Going to flower", goingToFlower);
            telemetry.addData("Follower busy", follower.isBusy());

            if (selectedFlower >= 0 && flowerTarget != null) {
                telemetry.addData("Selected flower", FLOWER_NAMES[selectedFlower]);
                telemetry.addData("Target X (in)", flowerTarget.x());
                telemetry.addData("Target Y (in)", flowerTarget.y());
                telemetry.addData("Target heading (deg)", Math.toDegrees(flowerTarget.heading()));
            }

            telemetry.addData("Forward", forward);
            telemetry.addData("Strafe", lateral);
            telemetry.addData("Turn", turn);

            telemetry.addData("Intake", intake);
            telemetry.addData("Outtake", outtake);
            telemetry.addData("Spit", spit);

            telemetry.addData("Front left", robot.lf.getPower());
            telemetry.addData("Back left", robot.lb.getPower());
            telemetry.addData("Front right", robot.rf.getPower());
            telemetry.addData("Back right", robot.rb.getPower());

            telemetry.addData("Robot X", robotPose.x());
            telemetry.addData("Robot Y", robotPose.y());
            telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));

            telemetry.update();
        }
    }

    private void goToNearestFlower() {
        Pose current = follower.pose();

        // Check current pos of robot
        if (current == null
                || !Double.isFinite(current.x())
                || !Double.isFinite(current.y())
                || !Double.isFinite(current.heading())) {
            return;
        }

        double closestDistance = Double.MAX_VALUE;
        selectedFlower = -1;

        // Find the closest flower center.
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
        double flowerHeading = Math.toRadians(FLOWERS[selectedFlower][2]);

        // Offset the robot's destination so the intake meets the flower.
        double targetX = flowerX - INTAKE_FORWARD_IN * Math.cos(flowerHeading) + INTAKE_LEFT_IN * Math.sin(flowerHeading);

        double targetY = flowerY - INTAKE_FORWARD_IN * Math.sin(flowerHeading) - INTAKE_LEFT_IN * Math.cos(flowerHeading);

        flowerTarget = new Pose(targetX, targetY, flowerHeading);

        // Avoid creating a nearly zero-length path.
        // This does not correct heading if already at the target location.
        if (Math.hypot(targetX - current.x(), targetY - current.y()) < 0.1) {
            return;
        }

        Pose start = new Pose(current.x(), current.y(), current.heading());

        Path path = line(start, flowerTarget).linear(start, flowerTarget);

        follower.follow(path);
        goingToFlower = true;
    }
}