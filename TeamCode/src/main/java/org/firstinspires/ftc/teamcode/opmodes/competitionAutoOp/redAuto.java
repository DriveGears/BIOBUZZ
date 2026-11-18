package org.firstinspires.ftc.teamcode.opmodes.competitionAutoOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "RedAuto (Upward Cell)", group = "Competition")
public class redAuto extends OpMode {
    private Follower follower;
    private Hardware robot;
    private turretSubsystem turret;
    private shooterSubsystem shooter;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private boolean shotSkipped;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    //Poses
    private final Pose startPose = poseFactory.of(36.6, 9.2, 179.999); //backplate on top of outer hive bar
//    private final Pose gardenPose = poseFactory.of(14, 9.2, 179.999);
//    private final Pose gardenToWFlowerControl = poseFactory.of(36.31, 53.68, 179.999);
private final Pose startToWFlowerControl = poseFactory.of(40.35, 34.81, 179.999);
    private final Pose WflowerPose = poseFactory.of(8, 52, 179.999);
    private final Pose NflowerPose = poseFactory.of(38, 129, 90);
    private final Pose WFlowerToNFlowerControl = poseFactory.of(49.4, 109.64, 90);
    private final Pose parkPose = poseFactory.of(8, 121, 0);

//    private final Pose returnPose = poseFactory.of(, , );

    // Path methods
//    private Path startToGarden() {
//        return line(startPose, gardenPose).linear(startPose, gardenPose);
//    }

    private Path startToWFlower(){
        return curve(startPose, startToWFlowerControl, WflowerPose).linear(startPose, WflowerPose);
    }

    private Path WFlowerToNFlower(){
        return curve(WflowerPose, WFlowerToNFlowerControl, NflowerPose).linear(WflowerPose, NflowerPose);
    }


    private Path park(){
        return line(NflowerPose, parkPose).linear(NflowerPose, parkPose);
    }

    private Path returnPose(){
        return line(parkPose, startPose).linear(parkPose, startPose);
    }


    private Command autoRoutine() {
        return sequential(
                // 1. At start pose: turn on intake & outtake, enable auto-aim, wait until shooter & turret ready (with 3s safety timeout), run transfer for 3.5s
                instant(() -> {
                    shooter.setTargetRpm(shooterSubsystem.COMPETITION_RPM);
                    shooter.setShooterEnabled(true);
                    robot.intake.setPower(0.85);
                    turret.setAutoAimEnabled(true);
                }),
                race(
                        waitUntil(() -> shooter.isAtSpeed() && turret.isAimReady()),
                        waitMs(3000)
                ),
                instant(() -> robot.transfer.setPower(1.0)),
                waitMs(3500),

                // 2. Move to Wflower: turn off transfer, keep intake & outtake on, disable auto-aim during transit
                instant(() -> {
                    robot.transfer.setPower(0);
                    turret.setAutoAimEnabled(false);
                }),
                follow(follower, startToWFlower()),

                // 3. Stop at Wflower for 3.5 seconds
                waitMs(3500),

                // 4. Move to N flower: turn intake OFF, enable auto-aim
                instant(() -> {
                    robot.intake.setPower(0);
                    turret.setAutoAimEnabled(true);
                }),
                follow(follower, WFlowerToNFlower()),

                // 5. At N flower position: wait until shooter & turret ready, pause briefly to settle, run transfer for 3.5s
                race(
                        waitUntil(() -> shooter.isAtSpeed() && turret.isAimReady()),
                        waitMs(3000)
                ),
                waitMs(300), // settle time to ensure precise angle tracking
                instant(() -> robot.transfer.setPower(1.0)),
                waitMs(3500),

                // 6. After 3.5s: turn intake ON, wait until shooter & turret ready, pause briefly to settle, run transfer for 3.5s
                instant(() -> robot.intake.setPower(0.85)),
                race(
                        waitUntil(() -> shooter.isAtSpeed() && turret.isAimReady()),
                        waitMs(3000)
                ),
                waitMs(300), // settle time to ensure precise angle tracking
                instant(() -> robot.transfer.setPower(1.0)),
                waitMs(3500),

                // 7. Go to park
                instant(() -> {
                    robot.transfer.setPower(0);
                    robot.intake.setPower(0);
                    shooter.setShooterEnabled(false);
                    turret.setAutoAimEnabled(false);
                }),
                follow(follower, park())
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
        turret.setAlliance(turretSubsystem.Alliance.RED);
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
