package org.firstinspires.ftc.teamcode.opmodes.competitionAutoOp;

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
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "RedAuto (Upward Cell)", group = "Competition")
public class Red_UpwardCell extends OpMode {
    private Follower follower;
    private Hardware robot;
    private turretSubsystem turret;
    private shooterSubsystem shooter;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private boolean shotSkipped;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    //Poses
    private final Pose startPose = poseFactory.of(32.0, 9.1, 180);
    private final Pose flowerLineupPose = poseFactory.of(7.6, 58.0, 195.3);
    private final Pose flowerPose = poseFactory.of(3.7, 55.9, 189.5);
    private final Pose flower2LineupPose = poseFactory.of(32.0, 123, 80.5);
    private final Pose flowerPose2 = poseFactory.of(33.0, 130.9, 81.3);
    private final Pose parkPose = poseFactory.of(1.3, 118.6, 2.0);

    // Path methods
    private Path startToFlowerLineup() {
        return line(startPose, flowerLineupPose).linear(startPose, flowerLineupPose);
    }

    private Path flowerLineupToFlower() {
        return line(flowerLineupPose, flowerPose).linear(flowerLineupPose, flowerPose);
    }

    private Path flowerToLineup(){
        return line(flowerPose, flower2LineupPose).linear(flowerPose, flower2LineupPose);
    }

    private Path lineupToFlower2(){
        return line(flower2LineupPose, flowerPose2).linear(flower2LineupPose, flowerPose2);
    }

    private Path park(){
        return line(flowerPose2, parkPose).linear(flowerPose2, parkPose);
    }

    private Command collectFromFlower() {
        return sequential(
                instant(() -> robot.intake.setPower(0.85)),
                waitMs(2000)
        );
    }

    private Command shootAtUpwardCell(long feedMs) {
        return sequential(
                instant(() -> {
                    shotSkipped = false;
                    shooter.setTargetRpm(shooterSubsystem.COMPETITION_RPM);
                    shooter.setShooterEnabled(true);
                    turret.setAutoAimEnabled(true);
                }),
                race(
                        waitUntil(() -> shooter.isAtSpeed() && turret.isAimReady()),
                        waitMs(2800)
                ),
                waitMs(200),
                instant(() -> robot.transfer.setPower(1.0)),
                waitMs(feedMs),
                instant(() -> {
                    robot.transfer.setPower(0);
                    turret.setAutoAimEnabled(false);
                })
        );
    }

    private Command autoRoutine() {
        return sequential(
                // 1. At start pose: turn on intake & outtake, enable auto-aim, wait until shooter & turret ready, run transfer for 3s
                instant(() -> {
                    shooter.setTargetRpm(shooterSubsystem.COMPETITION_RPM);
                    shooter.setShooterEnabled(true);
                    robot.intake.setPower(0.85);
                    turret.setAutoAimEnabled(true);
                }),
                race(
                        waitUntil(() -> shooter.isAtSpeed() && turret.isAimReady()),
                        waitMs(2800)
                ),
                instant(() -> robot.transfer.setPower(1.0)),
                waitMs(2800),

                // 2. Move to flower lineup: turn off transfer, keep intake & outtake on, disable auto-aim during transit
                instant(() -> {
                    robot.transfer.setPower(0);
                    turret.setAutoAimEnabled(false);
                }),
                follow(follower, startToFlowerLineup()),

                // 3. Settle at lineup
                waitMs(200),

                // 4. Move to flower pose: enable auto-aim
                instant(() -> {
                    turret.setAutoAimEnabled(true);
                }),
                follow(follower, flowerLineupToFlower()),

                // 5. Collect from flower
                collectFromFlower(),

                // 6. Flower 2 lineup & shoot
                instant(() -> turret.setAutoAimEnabled(false)),
                follow(follower, flowerToLineup()),
                waitMs(200),
                shootAtUpwardCell(2800),

                // 7. Flower 2 collect & shoot
                instant(() -> turret.setAutoAimEnabled(false)),
                follow(follower, lineupToFlower2()),
                collectFromFlower(),
                shootAtUpwardCell(2800),

                // 8. Park
                instant(() -> {
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
        robot.setTurretCurrentPositionAsZero(); // Zero turret at init
        follower = Constants.create(hardwareMap);
        turret = new turretSubsystem(robot);
        shooter = new shooterSubsystem(robot);
        turret.setAlliance(turretSubsystem.Alliance.RED);
        shooter.setTargetRpm(shooterSubsystem.COMPETITION_RPM);
        follower.setPose(startPose);
        follower.update();
        loopTimer.reset();
        telemetry.addData("Status", "Ready - Keep robot still before Play");
        telemetry.addData("Turret Ticks", robot.getTurretPositionTicks());
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
        telemetry.addData("Turret Ticks (Manual Test)", robot.getTurretPositionTicks());
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
        if (turret.hasDistance()) {
            shooter.setTargetForDistance(turret.getShooterDistanceIn());
        } else {
            shooter.setTargetRpm(shooterSubsystem.COMPETITION_RPM);
        }
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
