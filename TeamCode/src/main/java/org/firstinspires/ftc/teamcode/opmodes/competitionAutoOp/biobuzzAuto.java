package org.firstinspires.ftc.teamcode.opmodes.competitionAutoOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

/** Abstract base class for alliance-specific competition Autonomous routines. */
public abstract class biobuzzAuto extends OpMode {
    protected static final int AIM_WAIT_MS = 2400;
    protected static final int SHOT_SETTLE_MS = 200;
    protected static final int SHOT_FEED_MS = 1900;
    protected static final int COLLECT_MS = 2000;
    protected static final int SETTLE_MS = 150;

    protected static final double INTAKE_POWER = 0.85;
    protected static final int DRIVE_TO_FLOWER1_LINEUP_MS = 3500;
    protected static final int DRIVE_TO_FLOWER1_MS = 1500;
    protected static final int DRIVE_TO_SHOOT2_MS = 4000;
    protected static final int DRIVE_TO_FLOWER2_LINEUP_MS = 2000;
    protected static final int DRIVE_TO_FLOWER2_MS = 1500;
    protected static final int DRIVE_TO_PARK_MS = 2800;

    protected Follower follower;
    protected Hardware robot;
    protected turretSubsystem turret;
    protected shooterSubsystem shooter;
    protected final ElapsedTime loopTimer = new ElapsedTime();
    protected boolean shotSkipped;
    protected boolean driveStalled;
    protected final PoseFactory poseFactory = PoseFactory.degrees();

    protected abstract turretSubsystem.Alliance competitionAlliance();

    protected Pose getStartPose() { return poseFactory.of(32.0, 9.1, 180); }
    protected Pose getFlowerLineupPose() { return poseFactory.of(7.6, 58.0, 195.3); }
    protected Pose getFlowerPose() { return poseFactory.of(3.0, 55.9, 189.5); }
    protected Pose getShoot2Pose() { return poseFactory.of(22.0, 122.0, 81.1); }
    protected Pose getFlower2LineupPose() { return poseFactory.of(35.35, 126.0, 81.1); }
    protected Pose getFlowerPose2() { return poseFactory.of(33.3, 131, 80.04); }
    protected Pose getParkPose() { return poseFactory.of(2.0, 121.2, 359.7); }

    protected Path startToFlowerLineup() {
        return line(getStartPose(), getFlowerLineupPose()).linear(getStartPose(), getFlowerLineupPose());
    }
    protected Path flowerLineupToFlower() {
        return line(getFlowerLineupPose(), getFlowerPose()).linear(getFlowerLineupPose(), getFlowerPose());
    }
    protected Path flowerToShoot2() {
        return line(getFlowerPose(), getShoot2Pose()).linear(getFlowerPose(), getShoot2Pose());
    }
    protected Path shoot2ToFlower2Lineup() {
        return line(getShoot2Pose(), getFlower2LineupPose()).linear(getShoot2Pose(), getFlower2LineupPose());
    }
    protected Path flower2LineupToFlower2(){
        return line(getFlower2LineupPose(), getFlowerPose2()).linear(getFlower2LineupPose(), getFlowerPose2());
    }
    protected Path park(){
        return line(getFlowerPose2(), getParkPose()).linear(getFlowerPose2(), getParkPose());
    }

    protected Command drive(Path path, int timeoutMs) {
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

    protected Command collectFromFlower() {
        return sequential(
                instant(() -> robot.intake.setPower(INTAKE_POWER)),
                waitMs(COLLECT_MS)
        );
    }

    protected Command shootAtUpwardCell(long feedMs) {
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

    protected Command autoRoutine() {
        return sequential(
                shootAtUpwardCell(SHOT_FEED_MS),
                instant(() -> {
                    robot.transfer.setPower(0);
                    turret.setAutoAimEnabled(false);
                }),
                drive(startToFlowerLineup(), DRIVE_TO_FLOWER1_LINEUP_MS),
                waitMs(SETTLE_MS),
                instant(() -> turret.setAutoAimEnabled(true)),
                drive(flowerLineupToFlower(), DRIVE_TO_FLOWER1_MS),
                collectFromFlower(),
                instant(() -> turret.setAutoAimEnabled(false)),
                drive(flowerToShoot2(), DRIVE_TO_SHOOT2_MS),
                waitMs(SETTLE_MS),
                shootAtUpwardCell(SHOT_FEED_MS + 1000),
                instant(() -> turret.setAutoAimEnabled(false)),
                drive(shoot2ToFlower2Lineup(), DRIVE_TO_FLOWER2_LINEUP_MS),
                waitMs(SETTLE_MS),
                instant(() -> turret.setAutoAimEnabled(true)),
                drive(flower2LineupToFlower2(), DRIVE_TO_FLOWER2_MS),
                collectFromFlower(),
                shootAtUpwardCell(SHOT_FEED_MS + 1000),
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
        robot.setTurretCurrentPositionAsZero();
        follower = Constants.create(hardwareMap);
        turret = new turretSubsystem(robot);
        shooter = new shooterSubsystem(robot);
        turret.setAlliance(competitionAlliance());
        shooter.setTargetRpm(shooterSubsystem.COMPETITION_RPM);
        follower.setPose(getStartPose());
        follower.update();
        loopTimer.reset();
    }

    @Override
    public void init_loop() {
        turret.setAlliance(competitionAlliance());
        follower.update();
        telemetry.addData("Status", "Ready - Keep robot still before Play");
        telemetry.addData("Alliance", turret.getAlliance());
        telemetry.addData("Turret Ticks", robot.getTurretPositionTicks());
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
        telemetry.addData("Transfer", robot.transfer.getPower() != 0 ? "FEEDING" : "OFF");
        telemetry.addData("Last Drive", driveStalled ? "TIMED OUT" : "ok");
        telemetry.addData("Shot", shotSkipped ? "SKIPPED" : "armed/running");
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
