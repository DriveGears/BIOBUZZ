package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;
import static com.pedropathing.ivy.commands.Commands.waitMs;

@Autonomous
public class AUTO extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    // Changed 180 and 185 to 179 to prevent the 180-degree angle wrap-around bug
    private final Pose startPose = p.of(56, 8, 179);
    private final Pose flowerPose = p.of(8.7, 8.6, 179);
    private final Pose control2Pose = p.of(31.7, 118.4, 179);
    private final Pose score2Pose = p.of(56.9, 117.7, 110);
    private final Pose flower2Pose = p.of(48.7, 130.7, 110);
    private final Pose control3Pose = p.of(46, 119.6, 110);
    private final Pose parkPose = p.of(8.4, 114.6, 0);

    private Path flower() {
        return line(startPose, flowerPose).linear(startPose, flowerPose);
    }

    private Path startToScore2() {
        return curve(flowerPose, control2Pose, score2Pose).linear(flowerPose, score2Pose);
    }

    private Path flower2() {
        return line(score2Pose, flower2Pose).linear(score2Pose, flower2Pose);
    }

    private Path park() {
        return curve(flower2Pose, control3Pose, parkPose).linear(flower2Pose, parkPose);
    }

    private Command autoRoutine() {
        return sequential(
                // Wait 5 seconds at the starting line before moving
                waitMs(5000),

                // Path 2 -> Hold position for 2 seconds
                follow(follower, flower()),
                race(hold(follower), waitMs(2000)),

                // Path 3 -> Hold position for 4 seconds
                follow(follower, startToScore2()),
                race(hold(follower), waitMs(4000)),

                // Path 4 -> Hold position for 3 seconds
                follow(follower, flower2()),
                race(hold(follower), waitMs(3000)),

                // Path 5 -> Hold position for 6 seconds
                follow(follower, park()),
                race(hold(follower), waitMs(6000))
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    @Override
    public void init_loop() {
        // Keeps the Pinpoint and follower updated while waiting on the field
        follower.update();
        telemetry.addData("Status", "Ready - Keep robot still before Play");
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}
