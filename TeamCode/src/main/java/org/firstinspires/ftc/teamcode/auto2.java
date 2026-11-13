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
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous
public class auto2 extends OpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    //Poses
    private final Pose startPose = poseFactory.of(32.0, 9.1, 90);
    private final Pose flowerPose = poseFactory.of(14.7, 47.0, 179);
    private final Pose flowerPose2 = poseFactory.of(47.2, 130.7, 90);
    private final Pose parkPose = poseFactory.of(8.3, 108.2, -33);

    private final Pose returnPose = poseFactory.of(32.0, 9.1, 90);

    // Path methods
    private Path startToScore() {
        return line(startPose, flowerPose).linear(startPose, flowerPose);
    }

    private Path flowerToScore(){
        return line(flowerPose, flowerPose2).linear(flowerPose, flowerPose2);
    }

    private Path park(){
        return line(flowerPose2, parkPose).linear(flowerPose2, parkPose);
    }

    private Path returnPose(){
        return line(parkPose, startPose).linear(parkPose, startPose);
    }

    private Command autoRoutine() {
        return sequential(
                waitMs(3000),
                follow(follower, startToScore()),
                waitMs(2000), //intake from flower
                follow(follower, flowerToScore()),
                waitMs(3000), //outtake 4 then intake from flower
                follow(follower, park()),
                waitMs(1000),
                follow(follower, returnPose()) //only so i dont have to get up
        );
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
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
        // add your other methods needed in the loop here

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}
