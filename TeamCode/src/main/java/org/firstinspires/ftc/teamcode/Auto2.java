package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.shooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.turretSubsystem;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "Red Auto", group = "Competition")
public class Auto2 extends OpMode {
    private Follower follower;
    private Hardware robot;
    private turretSubsystem turret;
    private shooterSubsystem shooter;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private boolean shotSkipped;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    //Poses
    private final Pose startPose = poseFactory.of(39.2, 8.5, -90);
    private final Pose scorePose = poseFactory.of(39.2, 12, -90);
    private final Pose controlPose = poseFactory.of(68.3, 18.5, -90);
    private final Pose flowerPose = poseFactory.of(7.9, 8.6, 179);
    private final Pose controlPose1 = poseFactory.of(37.5, 56.2, 179);
    private final Pose controlPose2 = poseFactory.of(2.2, 117.3, 179);
    private final Pose score2Pose = poseFactory.of(56.8, 117.3, 90);
    private final Pose flowerPose2 = poseFactory.of(56.5, 132.5, 90);
    private final Pose controlPose3 = poseFactory.of(5.56, 122.3, 179);
    private final Pose controlPose4 = poseFactory.of(35.8, 34, 179);
    private final Pose flowerPose3 = poseFactory.of(8.5, 56.5, 179);
    private final Pose controlPose5 = poseFactory.of(17.3, 84.4, 179);
    private final Pose parkPose = poseFactory.of(8.4, 118.3, 179);

    private final Pose returnPose = poseFactory.of(32.0, 9.1, 90);

    // Path methods
    private Path startToScore() {
        return line(startPose, scorePose).linear(startPose, scorePose);
    }
    private Path scoreToFlower() {
        return curve(scorePose, controlPose, flowerPose).linear(scorePose, flowerPose);
    }

    private Path flowerToScore(){
        return curve(flowerPose, controlPose1, controlPose2, scorePose).linear(flowerPose, scorePose);
    }

    private Path scoreToFlowerToScore(){
        return line(score2Pose, flowerPose2).linear(score2Pose, flowerPose2);
    }

    private Path flowerToFlower(){
        return curve(flowerPose2, controlPose3, controlPose4, flowerPose3).linear(flowerPose2, flowerPose3);
    }

    private Path park(){
        return curve(flowerPose3, controlPose5, parkPose).linear(flowerPose3, parkPose);
    }

    private Path returnPose(){
        return line(parkPose, startPose).linear(parkPose, startPose);
    }

    /** Collect for two seconds, then keep the intake on to retain the balls. */
    private Command collectFromFlower() {
        return sequential(
                instant(() -> robot.intake.setPower(0.85)),
                waitMs(2000)
        );
    }

    private Command shootAtUpwardCell() {
        return sequential(
                instant(() -> {
                    shotSkipped = false;
                    shooter.setTargetRpm(shooterSubsystem.COMPETITION_RPM);
                    shooter.setShooterEnabled(true);
                    robot.intake.setPower(0.85); // Turn on intake along with outtake
                    turret.setAutoAimEnabled(true);
                }),
                race(
                        waitUntil(() -> shooter.isAtSpeed() && turret.isAimReady()),
                        waitMs(2500)
                ),
                conditional(
                        () -> shooter.isAtSpeed() && turret.isAimReady(),
                        sequential(
                                waitMs(150), // Brief delay (a few ms) so intake & outtake lead the transfer
                                instant(() -> robot.transfer.setPower(1.0)),
                                waitMs(3000),
                                instant(() -> {
                                    robot.transfer.setPower(0);
                                    robot.intake.setPower(0.85);
                                })
                        ),
                        instant(() -> shotSkipped = true)
                ),
                instant(() -> {
                    robot.transfer.setPower(0);
                    robot.intake.setPower(0);
                    shooter.setShooterEnabled(false);
                    turret.setAutoAimEnabled(false);
                })
        );
    }

    private Command autoRoutine() {
        return sequential(
                // 1. Turn on outtake at the very beginning
                instant(() -> {
                    shooter.setTargetRpm(2900);
                    shooter.setShooterEnabled(true);
                }),
                // 2. Wait until outtake reaches 2100 RPM
                waitUntil(() -> shooter.getActualRpm() >= 2900),
                // 3. Turn on intake and transfer
                instant(() -> {
                    robot.intake.setPower(0);
                    robot.transfer.setPower(1.0);
                }),
                waitMs(1500),
                // 4. Turn OFF outtake and transfer while moving from startPose to flowerPose, keeping intake ON
                instant(() -> {
                    shooter.setShooterEnabled(true);
                    robot.transfer.setPower(0);
                    robot.intake.setPower(0);
                }),
                follow(follower, startToScore()),
                follow(follower, scoreToFlower()),
                collectFromFlower(),
                follow(follower, flowerToScore()),
                shootAtUpwardCell(),
                follow(follower, scoreToFlowerToScore()),
                collectFromFlower(),
                shootAtUpwardCell(),
                follow(follower, flowerToFlower()),
                collectFromFlower(),
                follow(follower, park())
//                waitMs(1000),
//                follow(follower, returnPose()) //only so i dont have to get up
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        Hardware.resetInstance();
        robot = Hardware.getInstance(hardwareMap);
        follower = Constants.create(hardwareMap);
        turret = new turretSubsystem(robot);
        shooter = new shooterSubsystem(robot);
        turret.setAlliance(turretSubsystem.Alliance.BLUE);
        shooter.setTargetRpm(shooterSubsystem.COMPETITION_RPM);
        follower.setPose(startPose);
        follower.update();
        loopTimer.reset();
        telemetry.addData("Status", "Ready - Keep robot still before Play");
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }

    @Override
    public void init_loop() {
        turret.setAlliance(turretSubsystem.Alliance.RED);
        follower.update();
        telemetry.addData("Status", "Ready - Keep robot still before Play");
        telemetry.addData("Alliance", turret.getAlliance());
        telemetry.addLine("INIT selection: B = RED | X = BLUE");
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }

    @Override
    public void start() {
        loopTimer.reset();
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        double dtSeconds = loopTimer.seconds();
        loopTimer.reset();

        Scheduler.execute();
        follower.update();
        shooter.update(dtSeconds);
        turret.update(dtSeconds);

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("Alliance / CELL", "%s / %s",
                turret.getAlliance(), turret.getSelectedClusterName());
        telemetry.addData("CELL Roll", turret.getSelectedClusterRollDeg());
        telemetry.addData("Aim Ready", turret.isAimReady());
        telemetry.addData("Shooter RPM", "target=%.0f actual=%.0f",
                shooter.getTargetRpm(), shooter.getActualRpm());
        telemetry.addData("Intake", robot.intake.getPower() != 0
                ? "ON - RETAINING" : "OFF");
        telemetry.addData("Transfer", robot.transfer.getPower() != 0
                ? "FEEDING" : "OFF");
        telemetry.addData("Shot", shotSkipped
                ? "SKIPPED - aim/RPM not ready" : "armed/running");
        telemetry.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        if (robot != null) {
            robot.intake.setPower(0);
            robot.transfer.setPower(0);
        }
        if (follower != null) follower.stop();
        if (turret != null) turret.stop();
        if (shooter != null) shooter.stop();
    }
}
