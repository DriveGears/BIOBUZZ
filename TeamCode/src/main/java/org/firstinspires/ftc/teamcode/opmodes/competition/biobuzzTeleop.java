package org.firstinspires.ftc.teamcode.opmodes.competition;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import static com.pedropathing.api.Paths.line;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.shooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.turretSubsystem;

@TeleOp(name = "Biobuzz Teleop", group = "TeleOp")
public class biobuzzTeleop extends LinearOpMode {

    private Follower follower;
    private turretSubsystem turret;
    private shooterSubsystem shooter;
    private ElapsedTime loopTimer;

    private boolean intakeToggle = false;
    private boolean outtakeToggle = false;
    private boolean lastRightTrigger = false;
    private boolean lastLeftTrigger = false;
    private boolean lastX = false;
    private boolean goingToFlower = false;

    // Estimated flower centers: X/Y in inches and heading in degrees.
    private static final double[][] FLOWERS = {
            {0, 48, 180},   // West
            {48, 144, 90},  // North
            {144, 96, 0},   // East
            {96, 0, 270}    // South
    };

    private static final String[] FLOWER_NAMES = {
            "West", "North", "East", "South"
    };

    // Measured from the tracked robot pose to the intake. Positive side is left.
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
        shooter = new shooterSubsystem(robot);
        loopTimer = new ElapsedTime();

        follower.setPose(new Pose(
                56.0,
                8.0,
                Math.toRadians(0)
        ));
        turret.setAlliance(turretSubsystem.Alliance.BLUE);
        shooter.setTargetRpm(shooterSubsystem.COMPETITION_RPM);

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.b) turret.setAlliance(turretSubsystem.Alliance.RED);
            if (gamepad1.x) turret.setAlliance(turretSubsystem.Alliance.BLUE);

            follower.update();
            telemetry.addLine("Robot Initialized with Turret & Robot-Centric Drive");
            telemetry.addData("Alliance", turret.getAlliance());
            telemetry.addLine("During INIT: B = RED alliance | X = BLUE alliance");
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

        follower.update();
        loopTimer.reset();
        lastX = gamepad1.x;
        lastRightTrigger = gamepad1.right_trigger > 0.5;
        lastLeftTrigger = gamepad1.left_trigger > 0.5;

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

            boolean xPressed = gamepad1.x && !lastX;
            lastX = gamepad1.x;

            if (gamepad1.a && goingToFlower) {
                follower.stop();
                goingToFlower = false;
            }

            if (xPressed && !goingToFlower && !gamepad1.a) {
                goToNearestFlower();
            }

            if (spit) {
                robot.intake.setPower(-0.7);
            } else {
                robot.intake.setPower(intakeToggle ? 0.7 : 0.0);
            }

            shooter.setShooterEnabled(outtakeToggle);
            shooter.update(dtSeconds);
            boolean transferFeeding = outtakeToggle && shooter.isAtSpeed();
            if (transferFeeding) {
                robot.transfer.setPower(0.8);
            } else {
                robot.transfer.setPower(0);
            }

            // Turret Auto-Aim (Hold Left Bumper for Auto-Aim, D-Pad L/R for manual)
            boolean autoAim = gamepad1.left_bumper;
            turret.setAutoAimEnabled(autoAim);

            double manualTurretCommand = (gamepad1.dpad_right ? 1.0 : 0.0)
                    - (gamepad1.dpad_left ? 1.0 : 0.0);
            turret.setManualPower(manualTurretCommand * 0.25);
            turret.update(dtSeconds);

            // Pedro owns the drivetrain while the flower macro is active.
            if (!goingToFlower) {
                double denominator = Math.max(Math.abs(forward) + Math.abs(lateral) + Math.abs(turn), 1.0);
                robot.lf.setPower((forward + lateral + turn) / denominator);
                robot.lb.setPower((forward - lateral + turn) / denominator);
                robot.rf.setPower((forward - lateral - turn) / denominator);
                robot.rb.setPower((forward + lateral - turn) / denominator);
            }

            follower.update();

            if (goingToFlower && !follower.isBusy()) {
                // Leave Pedro idle so it cannot keep writing drive powers after handoff.
                follower.stop();
                goingToFlower = false;
            }

            Pose robotPose = follower.pose();

            telemetry.addData("Turret Mode", autoAim ? "AUTO-AIM" : "MANUAL");
            telemetry.addData("Target Visible", turret.hasTarget() ? "YES (ID: " + turret.getSelectedTagId() + ")" : "NO");
            telemetry.addData("Alliance / CELL", "%s / %s",
                    turret.getAlliance(), turret.getSelectedClusterName());
            telemetry.addData("CELL Roll", Double.isFinite(turret.getSelectedClusterRollDeg())
                    ? String.format("%.1f deg (UP)", turret.getSelectedClusterRollDeg())
                    : "no scorable CELL");
            telemetry.addData("Prediction Lead", "%.2f deg", turret.getPanLeadDeg());
            telemetry.addData("Aim Ready?", turret.isAimReady() ? "READY" : "ALIGNING");
            telemetry.addData("Intake Toggle", intakeToggle ? "ON" : "OFF");
            telemetry.addData("Outtake Toggle", outtakeToggle ? "ON" : "OFF");
            telemetry.addData("Shooter RPM", "target=%.0f actual=%.0f %s",
                    shooter.getTargetRpm(), shooter.getActualRpm(),
                    !outtakeToggle ? "OFF"
                            : shooter.isAtSpeed() ? "READY" : "SPINNING UP");
            telemetry.addData("Transfer", !outtakeToggle ? "OFF"
                    : transferFeeding ? "FEEDING" : "WAITING FOR RPM");
            telemetry.addData("Flower Macro", goingToFlower ? "RUNNING" : "IDLE");
            if (selectedFlower >= 0 && flowerTarget != null) {
                telemetry.addData("Selected Flower", FLOWER_NAMES[selectedFlower]);
                telemetry.addData("Flower Target X", flowerTarget.x());
                telemetry.addData("Flower Target Y", flowerTarget.y());
                telemetry.addData("Flower Target Heading", Math.toDegrees(flowerTarget.heading()));
            }
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

        if (selectedFlower < 0) {
            return;
        }

        double flowerX = FLOWERS[selectedFlower][0];
        double flowerY = FLOWERS[selectedFlower][1];
        double flowerHeading = Math.toRadians(FLOWERS[selectedFlower][2]);

        // Stop with the intake, rather than the tracked robot center, at the flower.
        double targetX = flowerX
                - INTAKE_FORWARD_IN * Math.cos(flowerHeading)
                + INTAKE_LEFT_IN * Math.sin(flowerHeading);
        double targetY = flowerY
                - INTAKE_FORWARD_IN * Math.sin(flowerHeading)
                - INTAKE_LEFT_IN * Math.cos(flowerHeading);

        flowerTarget = new Pose(targetX, targetY, flowerHeading);

        if (Math.hypot(targetX - current.x(), targetY - current.y()) < 0.1) {
            return;
        }

        Pose start = new Pose(current.x(), current.y(), current.heading());
        Path path = line(start, flowerTarget).linear(start, flowerTarget);

        follower.follow(path);
        goingToFlower = true;
    }

    private void addControlsTelemetry() {
        telemetry.addLine();
        telemetry.addLine("=== GAMEPAD 1 CONTROLS ===");
        telemetry.addLine("INIT only: B = RED alliance | X = BLUE alliance");
        telemetry.addLine("Left Stick: Drive/strafe | Right Stick X: Turn");
        telemetry.addLine("RT (tap): Toggle intake | RB (hold): Reverse intake");
        telemetry.addLine("LT (tap): Toggle outtake + transfer");
        telemetry.addLine("LB (hold): Turret auto-aim | D-pad L/R: Manual turret");
        telemetry.addLine("X: Nearest-flower macro | A: Cancel macro");
    }
}
