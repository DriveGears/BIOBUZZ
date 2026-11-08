package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.groups.Groups.sequential;

@Autonomous(name = "Auto Test", group = "Autonomous")
public class AUTO extends OpMode {

    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose startPose = poseFactory.of(58, 9, 180); // tuned starting pose with 90 deg heading
    private final Pose flowerPose = poseFactory.of(19.2, 50.3, 180);
    private final Pose parkPose = poseFactory.of(46.8, 118.3, 0);
    private String initError = null;

    private Path park() {
        return line(startPose, parkPose).constant(0);
    }

    private Path startToScore() {
        return line(startPose, flowerPose).constant(0);
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, startToScore()),
                //add mechanism commands here.
                follow(follower, park())
        );
    }

    @Override
    public void init() {
        try {
            Scheduler.reset();
            follower = Constants.create(hardwareMap);
            follower.setPose(startPose);
            follower.update();
            telemetry.addData("Status", "Initialized Successfully");
        } catch (Exception e) {
            initError = e.getClass().getSimpleName() + ": " + e.getMessage();
            telemetry.addData("CRITICAL INIT ERROR", initError);
            telemetry.addLine("Check Driver Station config names: 'odo', 'lf', 'rf', 'lb', 'rb'");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        if (initError == null && follower != null) {
            schedule(autoRoutine());
        }
    }

    @Override
    public void loop() {
        if (initError != null) {
            telemetry.addData("CRITICAL ERROR", initError);
            telemetry.update();
            return;
        }

        follower.update();
        Scheduler.execute();

        telemetry.addData("Follower", "Running");
        if (follower != null) {
            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());
            telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        }
        telemetry.update();
    }
}
