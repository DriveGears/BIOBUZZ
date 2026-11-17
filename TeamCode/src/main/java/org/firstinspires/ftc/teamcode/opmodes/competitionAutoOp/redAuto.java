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
    private final Pose startPose = poseFactory.of(37.6, 9.2, 179.999); //backplate on top of outer hive bar
    private final Pose gardenPose = poseFactory.of(14, 9.2, 179.999);
    private final Pose gardenToWFlowerControl = poseFactory.of(36.31, 53.68, 179.999);
    private final Pose WflowerPose = poseFactory.of(20, 52, 179.999);
    private final Pose NflowerPose = poseFactory.of(40, 126, 90);
    private final Pose WFlowerToNFlowerControl = poseFactory.of(28.2, 108.68, 90);
    private final Pose parkPose = poseFactory.of(11, 121, 90);

//    private final Pose returnPose = poseFactory.of(, , );

    // Path methods
    private Path startToGarden() {
        return line(startPose, gardenPose).linear(startPose, gardenPose);
    }

    private Path gardenToWFlower(){
        return curve(gardenPose, gardenToWFlowerControl, WflowerPose).linear(gardenPose, WflowerPose);
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

    /** Collect for two seconds, then keep the intake on to retain the balls. */
//    private Command collectFromFlower() {
//        return sequential(
//                instant(() -> robot.intake.setPower(0.85)),
//                waitMs(2000)
//        );
//    }

//    private Command shootAtUpwardCell() {
//        return sequential(
//                instant(() -> {
//                    shotSkipped = false;
//                    shooter.setTargetRpm(shooterSubsystem.COMPETITION_RPM);
//                    shooter.setShooterEnabled(true);
//                    robot.intake.setPower(0.85); // Turn on intake along with outtake
//                    turret.setAutoAimEnabled(true);
//                }),
//                race(
//                        waitUntil(() -> shooter.isAtSpeed() && turret.isAimReady()),
//                        waitMs(2500)
//                ),
//                conditional(
//                        () -> shooter.isAtSpeed() && turret.isAimReady(),
//                        sequential(
//                                waitMs(150), // Brief delay (a few ms) so intake & outtake lead the transfer
//                                instant(() -> robot.transfer.setPower(1.0)),
//                                waitMs(3000),
//                                instant(() -> {
//                                    robot.transfer.setPower(0);
//                                    robot.intake.setPower(0);
//                                })
//                        ),
//                        instant(() -> shotSkipped = true)
//                ),
//                instant(() -> {
//                    robot.transfer.setPower(0);
//                    robot.intake.setPower(0);
//                    shooter.setShooterEnabled(false);
//                    turret.setAutoAimEnabled(false);
//                })
//        );
//    }

    private Command autoRoutine() {
        return sequential(
                // 1. Turn on outtake at the very beginning
//                instant(() -> {
//                    shooter.setTargetRpm(300);
//                    shooter.setShooterEnabled(true);
//                }),
//                // 2. Wait until outtake reaches 2100 RPM
//                waitUntil(() -> shooter.getActualRpm() >= 2100),
//                // 3. Turn on intake and transfer
//                instant(() -> {
//                    robot.intake.setPower(0.85);
//                    robot.transfer.setPower(1.0);
//                }),
//                waitMs(1500),
//                // 4. Turn OFF outtake and transfer while moving from startPose to flowerPose, keeping intake ON
//                instant(() -> {
//                    shooter.setShooterEnabled(false);
//                    robot.transfer.setPower(0);
//                    robot.intake.setPower(0.85);
//                }),
                follow(follower, startToGarden()),
                waitMs(500),
//                collectFromFlower(),
                follow(follower, gardenToWFlower()),
                waitMs(2000),
//                shootAtUpwardCell(),
                follow(follower, WFlowerToNFlower()),
                waitMs(2000),
                follow(follower, park()),
                waitMs(1000),
                follow(follower, returnPose()) //only so i dont have to get up
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
