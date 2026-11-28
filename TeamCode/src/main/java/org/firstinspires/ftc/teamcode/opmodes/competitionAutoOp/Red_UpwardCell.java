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
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
@Disabled
@Autonomous(name = "RedAuto (Upward Cell)", group = "Competition")
public class Red_UpwardCell extends OpMode {
    private static final int AIM_WAIT_MS = 2400;
    private static final int SHOT_SETTLE_MS = 200;
    private static final int SHOT_FEED_MS = 1900;
    private static final int COLLECT_MS = 2000;
    private static final int SETTLE_MS = 150;

    private static final double INTAKE_POWER = 0.85;
    private static final int DRIVE_TO_FLOWER1_LINEUP_MS = 3500;
    private static final int DRIVE_TO_FLOWER1_MS = 1500;
    private static final int DRIVE_TO_SHOOT2_MS = 4000;
    private static final int DRIVE_TO_FLOWER2_LINEUP_MS = 2000;
    private static final int DRIVE_TO_FLOWER2_MS = 1500;
    private static final int DRIVE_TO_PARK_MS = 2800;

    private Follower follower;
    private Hardware robot;
    private turretSubsystem turret;
    private shooterSubsystem shooter;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private boolean shotSkipped;
    private boolean driveStalled;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    //Poses
    private final Pose startPose = poseFactory.of(32.0, 9.1, 180);
    private final Pose flowerLineupPose = poseFactory.of(7.6, 58.0, 195.3);
    private final Pose flowerPose = poseFactory.of(3.0, 55.9, 189.5);
    private final Pose shoot2Pose = poseFactory.of(22.0, 122.0, 81.1);
    private final Pose flower2LineupPose = poseFactory.of(35.35, 126.0, 81.1);
    private final Pose flowerPose2 = poseFactory.of(33.3, 131, 80.04);
    private final Pose parkPose = poseFactory.of(2.0, 121.2, 359.7);

    // Path methods
    private Path startToFlowerLineup() {
        return line(startPose, flowerLineupPose).linear(startPose, flowerLineupPose);
    }
    private Path flowerLineupToFlower() {
        return line(flowerLineupPose, flowerPose).linear(flowerLineupPose, flowerPose);
    }
    private Path flowerToShoot2() {
        return line(flowerPose, shoot2Pose).linear(flowerPose, shoot2Pose);
    }
    private Path shoot2ToFlower2Lineup() {
        return line(shoot2Pose, flower2LineupPose).linear(shoot2Pose, flower2LineupPose);
    }
    private Path flower2LineupToFlower2(){
        return line(flower2LineupPose, flowerPose2).linear(flower2LineupPose, flowerPose2);
    }
    private Path park(){
        return line(flowerPose2, parkPose).linear(flowerPose2, parkPose);
    }

    private Command drive(Path path, int timeoutMs) {
        return race(
                sequential(
                        follow(follower, path),
                        instant(() -> driveStalled = false)
                ),
                sequential(
                        waitMs(timeoutMs),
                        instant(() -> driveStalled = true)
                )
        );
    }

    private Command collectFromFlower() {
        return sequential(
                instant(() -> robot.intake.setPower(INTAKE_POWER)),
                waitMs(COLLECT_MS)
        );
    }

    private Command shootAtUpwardCell(long feedMs) {
        return sequential(
                instant(() -> {
                    shotSkipped = false;
                    robot.intake.setPower(INTAKE_POWER);
                    if (turret.hasDistance()) {
                        shooter.setTargetForDistance(turret.getShooterDistanceIn());
                    } else {
                        shooter.setTargetRpm(shooterSubsystem.COMPETITION_RPM);
                    }
                    shooter.setShooterEnabled(true);
                    turret.setAutoAimEnabled(true);
                }),
                race(
                        waitUntil(() -> shooter.isAtSpeed() && turret.isAimReady()),
                        waitMs(AIM_WAIT_MS)
                ),
                instant(() -> shotSkipped = !(shooter.isAtSpeed() && turret.isAimReady())),
                waitMs(SHOT_SETTLE_MS),
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
                // 1. At start pose: spin up, aim, shoot the preload.
                shootAtUpwardCell(SHOT_FEED_MS),

                // 2. Move to flower 1 lineup
                instant(() -> {
                    robot.transfer.setPower(0);
                    turret.setAutoAimEnabled(false);
                }),
                drive(startToFlowerLineup(), DRIVE_TO_FLOWER1_LINEUP_MS),
                waitMs(SETTLE_MS),

                // 3. Move to flower 1 pose & collect
                instant(() -> turret.setAutoAimEnabled(true)),
                drive(flowerLineupToFlower(), DRIVE_TO_FLOWER1_MS),
                collectFromFlower(),

                // 4. Move to shoot2Pose & shoot (1 second longer feed: SHOT_FEED_MS + 1000)
                instant(() -> turret.setAutoAimEnabled(false)),
                drive(flowerToShoot2(), DRIVE_TO_SHOOT2_MS),
                waitMs(SETTLE_MS),
                shootAtUpwardCell(SHOT_FEED_MS + 1000),

                // 5. Move to flower 2 lineup (lineup for flower 2)
                instant(() -> turret.setAutoAimEnabled(false)),
                drive(shoot2ToFlower2Lineup(), DRIVE_TO_FLOWER2_LINEUP_MS),
                waitMs(SETTLE_MS),

                // 6. Move to flower pose 2, collect & shoot (1 second longer feed: SHOT_FEED_MS + 1000)
                instant(() -> turret.setAutoAimEnabled(true)),
                drive(flower2LineupToFlower2(), DRIVE_TO_FLOWER2_MS),
                collectFromFlower(),
                shootAtUpwardCell(SHOT_FEED_MS + 1000),

                // 7. Park
                instant(() -> {
                    shooter.setShooterEnabled(false);
                    robot.intake.setPower(0);
                    turret.setAutoAimEnabled(false);
                }),
                drive(park(), DRIVE_TO_PARK_MS)
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
        telemetry.addData("Pollen", "Arrange Flat");
        telemetry.addData("Alliance", turret.getAlliance());
        telemetry.addData("Turret Ticks (Manual Test - 210)", robot.getTurretPositionTicks());
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
        telemetry.addData("Intake", "%.2f", robot.intake.getPower());
        telemetry.addData("Transfer", robot.transfer.getPower() != 0
                ? "FEEDING" : "OFF");
        telemetry.addData("Last Drive", driveStalled ? "TIMED OUT" : "ok");
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
