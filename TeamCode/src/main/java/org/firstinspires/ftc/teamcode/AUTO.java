package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.groups.Groups.sequential;
import com.pedropathing.paths.Path;



//p.of( x, y, heading)
@Autonomous
public class AUTO extends OpMode{
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose startPose = poseFactory.of(24, 24, 0); //needs to be tuned
    private final Pose scorePose = poseFactory.of(48, 48, 90);
    private final Pose parkPose = poseFactory.of(72, 48, 90);


    //other poses...
    private final Pose controlPose = poseFactory.of(36, 60, 45);

    private Path park() {
        return line(startPose, parkPose).linear(startPose, parkPose);
        // return curve(startPose, controlPose, parkPose).linear(startPose, parkPose);
    }


    private Path startToScore() {
        return line(startPose, scorePose).linear(startPose, scorePose);
        // return curve(startPose, controlPose, parkPose).linear(startPose, parkPose);
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, startToScore()),
                //add mechanism commands here.
                follow(follower, park())
        );
    }

        @Override
        public void init () {
            Scheduler.reset();
            follower = Constants.create(hardwareMap);
            follower.setPose(startPose);
            follower.update();
        }

        @Override
        public void start () {
//        schedule(follow(follower, park()));
            schedule(autoRoutine());
        }

        @Override
        public void loop () {
            follower.update();
            Scheduler.execute();
            // add your other methods needed in the loop here
        }
}
