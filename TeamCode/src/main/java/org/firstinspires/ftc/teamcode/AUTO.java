package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.opencv.imgproc.Imgproc.line;



//p.of( x, y, heading)
@Autonomous
public class AUTO extends OpMode{
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(56, 8, 90);
    private final Pose scorePose = p.of(56, 21.3, 90);
    private final Pose controlPose = p.of(51.3, 6.8, 180);
    private final Pose flowerPose = p.of(8.7, 8.6, 180);
    private final Pose control2Pose = p.of(31.7, 118.4, 180);
    private final Pose score2Pose = p.of(56.9, 117.7, 110);
    private final Pose flower2Pose = p.of(48.7, 130.7, 110);
    private final Pose control3Pose = p.of(46, 119.6, 110);
    private final Pose parkPose = p.of(8.4, 114.6, 0);


    private Path startToScore() {
        return line(startPose, scorePose).linear(startPose, scorePose);
    }

    private Path line(Pose startPose, Pose scorePose) {
        return line(startPose, scorePose);
    }


    private Path flower() {
        return curve(scorePose, controlPose, flowerPose).linear(scorePose, flowerPose);
        // return curve(startPose, controlPose, parkPose).linear(startPose, parkPose);
    }

    private Path startToScore2() {
        return curve(flowerPose, control2Pose, score2Pose).linear(flowerPose, score2Pose);
        // return curve(startPose, controlPose, parkPose).linear(startPose, parkPose);
    }


    private Path flower2() {
        return line(score2Pose, flower2Pose).linear(score2Pose, flower2Pose);
        // return curve(startPose, controlPose, parkPose).linear(startPose, parkPose);
    }
    private Path park() {
        return curve(flower2Pose, control3Pose, parkPose).linear(flower2Pose, parkPose);
        // return curve(startPose, controlPose, parkPose).linear(startPose, parkPose);
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, startToScore()),
                follow(follower, flower()),
                follow(follower, startToScore2()),
                follow(follower, flower2()),
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
            schedule(autoRoutine());
        }

        @Override
        public void loop () {
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
