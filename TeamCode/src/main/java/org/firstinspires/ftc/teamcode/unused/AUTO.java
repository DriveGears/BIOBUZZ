package org.firstinspires.ftc.teamcode.unused;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.conditional;

@Autonomous
@Disabled
public class AUTO extends OpMode {
    private Follower follower;
    private Hardware robot;
    private turretSubsystem turret;
    private shooterSubsystem shooter;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private boolean shotSkipped;
    private final PoseFactory p = PoseFactory.degrees();

    private final Pose startPose = p.of(55.6, 9.5, 179);
    private final Pose flowerPose = p.of(20.4, 77.2, 179);
    // West of the HIVE and facing east. Field-test this pose for frame clearance.
    private final Pose shootPose = p.of(18.0, 72.0, 0);

    private Path flower() {
        return line(startPose, flowerPose).linear(startPose, flowerPose);
    }

    private Path flowerToShoot() {
        return line(flowerPose, shootPose).linear(flowerPose, shootPose);
    }

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
                    turret.setAutoAimEnabled(true);
                }),
                race(
                        waitUntil(() -> shooter.isAtSpeed() && turret.isAimReady()),
                        waitMs(2500)
                ),
                conditional(
                        () -> shooter.isAtSpeed() && turret.isAimReady(),
                        sequential(
                                instant(() -> {
                                    robot.transfer.setPower(shooterSubsystem.TRANSFER_POWER);
                                }),
                                waitMs(1600),
                                instant(this::stopFeed)
                        ),
                        instant(() -> shotSkipped = true)
                ),
                instant(() -> {
                    stopFeed();
                    shooter.setShooterEnabled(false);
                    turret.setAutoAimEnabled(false);
                })
        );
    }

    private void stopFeed() {
        robot.transfer.setPower(0);
    }

    private Command autoRoutine() {
        return sequential(
                // Wait 1 second at the starting line before moving
                waitMs(1000),

                // Go to flower and hold position
                follow(follower, flower()),
                collectFromFlower(),
                follow(follower, flowerToShoot()),
                race(hold(follower), shootAtUpwardCell())
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
        telemetry.addData("Intake Power", robot.intake.getPower());
        telemetry.addData("Shot", shotSkipped ? "SKIPPED - aim/RPM not ready" : "armed/running");
        telemetry.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        if (robot != null) robot.transfer.setPower(0);
        if (follower != null) follower.stop();
        if (turret != null) turret.stop();
        if (shooter != null) shooter.stop();
        if (robot != null) robot.intake.setPower(0);
    }
}
