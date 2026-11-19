package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.tools.AutoLogger;
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
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "Red Auto", group = "Competition")
public class redAuto extends OpMode {
    private Follower follower;
    private Hardware robot;
    private turretSubsystem turret;
    private shooterSubsystem shooter;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private boolean shotSkipped;
    private final AutoLogger log = new AutoLogger();
    private final ElapsedTime runTimer = new ElapsedTime();
    private String step = "init";
    private Pose target = null;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    //Poses
    private final Pose startPose = poseFactory.of(32.0, 9.1, 90);
    private final Pose gardenPose = poseFactory.of(32.9, 35.5, 90);
    // Two-stage moves near the walls: the robot overshoots 4-7" when a long move ends at speed
    // (reached y=40 vs a 32.4 garden target, and y=-3 vs the start at the back wall).
    // Stopping ~10" short first and finishing with a short move keeps the overshoot to ~1".
    private final Pose gardenApproachPose = poseFactory.of(32.9, 25.5, 90);
    private final Pose startApproachPose = poseFactory.of(32.0, 19.5, 90);
    private final Pose flowerPose = poseFactory.of(77.6, 36.9, 89);
    // Line-up point before flowerPose2 so the front of the robot doesn't clip on approach.
    // Final approach into the first flower is a straight FORWARD drive (+y) from 12" behind it.
    // Approaching sideways along +x made the robot's right side hit the flower.
    private final Pose flowerLineupPose = poseFactory.of(77.6, 24.9, 89);
    // Shifted +3" in y (left, since the robot faces +x here): the log showed it reaching its target but sitting too far right of the second flower.
    private final Pose flower2LineupPose = poseFactory.of(138.8, 9.8, 2);
    private final Pose flowerPose2 = poseFactory.of(152.1, 10.9, 2);
    private final Pose parkPose = poseFactory.of(143.3, 42.4, 271);

    private final Pose returnPose = poseFactory.of(32.0, 9.1, 90);

    // Path methods
    private Path gardenApproachToGarden() {
        return line(gardenApproachPose, gardenPose).linear(gardenApproachPose, gardenPose);
    }

    private Path startToGardenApproach() {
        return line(startPose, gardenApproachPose).linear(startPose, gardenApproachPose);
    }

    private Path gardenToStartApproach() {
        return line(gardenPose, startApproachPose).linear(gardenPose, startApproachPose);
    }

    private Path startApproachToStart() {
        return line(startApproachPose, startPose).linear(startApproachPose, startPose);
    }

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

    /**
     * Spin up, then feed the transfer for feedMs. Does NOT wait for shooter.isAtSpeed() or aim lock:
     * the log showed RPM wobbling 2143-2229 against a 2100 target (outside the +/-32 RPM
     * stability window), so isAtSpeed() never went true and the transfer never ran.
     * Feeds once RPM is >= 90% of target, or after 2.5s regardless.
     * The follower is left in HOLD so the robot stays put (follower.stop() let it coast ~9" back).
     */
    private Command shootAtUpwardCell(long feedMs) {
        return sequential(
                instant(() -> {
                    shotSkipped = false;
                    shooter.setTargetRpm(shooterSubsystem.COMPETITION_RPM);
                    shooter.setShooterEnabled(true);
                    turret.setAutoAimEnabled(true);
                }),
                race(
                        waitUntil(() -> shooter.getActualRpm() >= 0.9 * shooterSubsystem.COMPETITION_RPM),
                        waitMs(2500)
                ),
                instant(() -> robot.transfer.setPower(shooterSubsystem.TRANSFER_POWER)),
                waitMs(feedMs),
                instant(() -> {
                    robot.transfer.setPower(0);
                    turret.setAutoAimEnabled(false);
                })
        );
    }

    /** Tags the log with the current step and the pose we are driving toward. */
    private Command mark(String name, Pose to) {
        return instant(() -> { step = name; target = to; });
    }

    private Command autoRoutine() {
        return sequential(
                // The 30s auto ran out during the last shot, so the 3s start wait is gone and the
                // flywheel spins up while driving (it needed ~2.2s from 0 at every outtake).
                instant(() -> {
                    robot.intake.setPower(0.85);
                    shooter.setTargetRpm(shooterSubsystem.COMPETITION_RPM);
                    shooter.setShooterEnabled(true);
                }),
                mark("startToGardenApproach", gardenApproachPose),
                follow(follower, startToGardenApproach()),
                waitMs(250),
                mark("startToGarden", gardenPose),
                follow(follower, gardenApproachToGarden()),
                mark("settleGarden", gardenPose),
                waitMs(300),
                mark("gardenToStartApproach", startApproachPose),
                follow(follower, gardenToStartApproach()),
                waitMs(250),
                mark("gardenToStart", startPose),
                follow(follower, startApproachToStart()),
                mark("settleStart", startPose),
                waitMs(300),
                mark("outtakeAtStart", startPose),
                shootAtUpwardCell(2500),
                mark("startToFlowerLineup", flowerLineupPose),
                follow(follower, startToFlowerLineup()),
                mark("settleLineup1", flowerLineupPose),
                waitMs(400), // let heading/position settle before the final approach
                mark("lineupToFlower", flowerPose),
                follow(follower, flowerLineupToFlower()),
                mark("collect", flowerPose),
                collectFromFlower(),
                mark("flowerToLineup", flower2LineupPose),
                follow(follower, flowerToLineup()),
                mark("settleLineup2", flower2LineupPose),
                waitMs(400), // heading was still ~10 deg off when arriving here
                // Empty the first flower's balls BEFORE taking in the second flower's.
                mark("outtakeAtLineup2", flower2LineupPose),
                shootAtUpwardCell(3000),
                mark("lineupToFlower2", flowerPose2),
                follow(follower, lineupToFlower2()),
                mark("collect2", flowerPose2),
                collectFromFlower(), // the second flower had no collect step, so it never took its balls in
                mark("shoot", flowerPose2),
                shootAtUpwardCell(4500), // longest feed so every ball is out
                instant(() -> shooter.setShooterEnabled(false)),
                mark("park", parkPose),
                follow(follower, park()),
                mark("return", returnPose),
                follow(follower, returnPose()), //only so i dont have to get up
                mark("done", returnPose)
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
        if (gamepad1.b) turret.setAlliance(turretSubsystem.Alliance.RED);
        if (gamepad1.x) turret.setAlliance(turretSubsystem.Alliance.BLUE);

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
        runTimer.reset();
        log.open();
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

        Pose cur = follower.pose();
        Pose tgt = target != null ? target : cur;
        log.row(runTimer.seconds(), step, cur.x(), cur.y(), Math.toDegrees(cur.heading()),
                tgt.x(), tgt.y(), Math.toDegrees(tgt.heading()), String.valueOf(follower.mode()),
                shooter.getTargetRpm(), shooter.getActualRpm(), turret.isAimReady(),
                robot.intake.getPower(), robot.transfer.getPower(), dtSeconds * 1000.0);

        telemetry.addData("Step", step);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("Alliance / CELL", "%s / %s",
                turret.getAlliance(), turret.getSelectedClusterName());
        telemetry.addData("CELL Roll", turret.getSelectedClusterRollDeg());
        telemetry.addData("Aim Ready", turret.isAimReady());
        telemetry.addData("Chassis Aim Assist", "DISABLED");
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
        log.close();
        Scheduler.reset();
        if (robot != null) {
            robot.intake.setPower(0);
            robot.transfer.setPower(0);
        }
        if (follower != null) follower.stop();
        if (turret != null) turret.stop();
        if (shooter != null) shooter.stop();
    }

    private void stopDrive() {
        robot.lf.setPower(0);
        robot.lb.setPower(0);
        robot.rf.setPower(0);
        robot.rb.setPower(0);
    }
}
