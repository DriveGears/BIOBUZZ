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
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;

@Autonomous
public class AUTO extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(56, 8, 90);
    private final Pose controlPose = p.of(51.3, 6.8, 180);
    private final Pose flowerPose = p.of(8.7, 8.6, 180);

    private Path flower() {
        return curve(startPose, controlPose, flowerPose).linear(startPose, flowerPose);
    }

    private Command autoRoutine() {
        return sequential(
                // Path 1 -> Follow path, then hold position for 4 seconds
                follow(follower, flower()),
                parallel(hold(follower), waitMs(4000))
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
        // Continuously update localization and IMU during the INIT phase
        follower.update();

        // Optional: Display coordinates on Driver Station during init to verify tracking
        telemetry.addData("Status", "Initialized & Ready");
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


//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.pedropathing.follower.Follower;
//import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
//import com.pedropathing.api.PoseFactory;
//import com.pedropathing.math.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.ivy.Command;
//import com.pedropathing.ivy.Scheduler;
//
//import static com.pedropathing.api.Paths.line;
//import static com.pedropathing.api.Paths.curve;
//import static com.pedropathing.ivy.Scheduler.schedule;
//import static com.pedropathing.ivy.groups.Groups.sequential;
//import static com.pedropathing.ivy.pedro.PedroCommands.follow;
//import static com.pedropathing.ivy.pedro.PedroCommands.hold;
//import static com.pedropathing.ivy.commands.Commands.waitMs;
//import static com.pedropathing.ivy.groups.Groups.parallel;
//
//@Autonomous
//public class AUTO extends OpMode {
//    private Follower follower;
//    private final PoseFactory p = PoseFactory.degrees();
//    private final Pose startPose = p.of(56, 8, 90);
////    private final Pose scorePose = p.of(56, 21.3, 90);
//    private final Pose controlPose = p.of(51.3, 6.8, 180);
//    private final Pose flowerPose = p.of(8.7, 8.6, 180);
//    private final Pose control2Pose = p.of(31.7, 118.4, 180);
//    private final Pose score2Pose = p.of(56.9, 117.7, 110);
////    private final Pose flower2Pose = p.of(48.7, 130.7, 110);
////    private final Pose control3Pose = p.of(46, 119.6, 110);
////    private final Pose parkPose = p.of(8.4, 114.6, 0);
//
////    private Path startToScore() {
////        // Correctly calls Pedro Pathing's line() now
////        return line(startPose, scorePose).linear(startPose, scorePose);
////    }
//
//    private Path flower() {
//        return curve(startPose, controlPose, flowerPose).linear(startPose, flowerPose);
//    }
//
////    private Path startToScore2() {
////        return curve(flowerPose, control2Pose, score2Pose).linear(flowerPose, score2Pose);
////    }
//
////    private Path flower2() {
////        return line(score2Pose, flower2Pose).linear(score2Pose, flower2Pose);
////    }
////
////    private Path park() {
////        return curve(flower2Pose, control3Pose, parkPose).linear(flower2Pose, parkPose);
////    }
//
//    private Command autoRoutine() {
//        return sequential(
//                // Path 1 -> Stop for 4 seconds
//                follow(follower, flower()),
//                parallel(hold(follower), waitMs(4000)));
//
////                // Path 2 -> Stop for 1 second
////                follow(follower, flower()),
////                parallel(hold(follower), waitMs(1000)),
////
////                // Path 3 -> Stop for 4 seconds
////                follow(follower, startToScore2()),
////                parallel(hold(follower), waitMs(4000)),
////
////                // Path 4 -> Stop for 3 seconds
////                follow(follower, flower2()),
////                parallel(hold(follower), waitMs(3000)),
////
////                // Path 5 -> Stop for 5.2 seconds
////                follow(follower, park()),
////                parallel(hold(follower), waitMs(5200))
////        );
//    }
//
//    @Override
//    public void init() {
//        Scheduler.reset();
//        follower = Constants.create(hardwareMap);
//        follower.setPose(startPose);
//        follower.update();
//    }
//
//    @Override
//    public void start() {
//        schedule(autoRoutine());
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        Scheduler.execute();
//
//        telemetry.addData("X", follower.pose().x());
//        telemetry.addData("Y", follower.pose().y());
//        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
//        telemetry.addData("Follower Mode", follower.mode());
//        telemetry.update();
//    }
//}
//
//
