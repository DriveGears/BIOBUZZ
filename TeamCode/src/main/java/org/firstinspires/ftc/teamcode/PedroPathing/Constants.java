package org.firstinspires.ftc.teamcode.PedroPathing;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("lf");
        c.frontRightName.set("rf");
        c.backLeftName.set("lb");
        c.backRightName.set("rb");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odo");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.644494604876661);
        c.yPodOffset.set(-1.5708831727035402);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.05);
                Controller secondaryTranslationalForward = Controller.proportional(0.12);
                Controller primaryTranslationalLateral = Controller.proportional(0.22);
                Controller secondaryTranslationalLateral = Controller.proportional(0.40);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.0));
                c.brake.set(Controller.proportionalFeedforward(0.0));

                c.headingFeedback.set(Controller.proportional(1.5));

                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.1, -0.01));
                c.linearBrakeCoefficients.set(Matrix.diag(0.05, 0.02));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0, 0.0));

                c.maxAchievableForwardVelocity.set(75.00153749433845);
                c.maxAchievableStrafeVelocity.set(56.89789011688528);
                c.naturalForwardDeceleration.set(39.85089153986496);
                c.naturalStrafeDeceleration.set(75.34984129868222);
            }
    );
//    public static ForesightConfig foresightConfig = new ForesightConfig(
//            c -> {
//                Controller primaryTranslationalForward = Controller.proportional(0.26242716951699363);
//                Controller secondaryTranslationalForward = Controller.proportional(0.09695982695341443);
//                Controller primaryTranslationalLateral = Controller.proportional(0.4896145091670653);
//                Controller secondaryTranslationalLateral = Controller.proportional(0.1808994783965974);
//
//                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
//                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
//
//                c.coast.set(Controller.proportionalFeedforward(0.009837355147650788));
//                c.brake.set(Controller.proportionalFeedforward(0.00836175187550317));
//
//                c.headingFeedback.set(Controller.proportional(6.0797651837762245));
//                c.headingBrakeCoefficients.set(Vector2D.cartesian(2.8758582471955627, -0.30930118980028654));
//
//                c.linearBrakeCoefficients.set(Matrix.diag(0.139468209897466, 0.05599364580000122));
//                c.quadraticBrakeCoefficients.set(Matrix.diag(5.9043725560968095E-5, 0.001845430824898247));
//
//                c.maxAchievableForwardVelocity.set(75.00153749433845);
//                c.maxAchievableStrafeVelocity.set(56.89789011688528);
//                c.naturalForwardDeceleration.set(39.85089153986496);
//                c.naturalStrafeDeceleration.set(75.34984129868222);
//            }
//    );
//    public static ForesightConfig foresightConfig = new ForesightConfig(
//            c -> {
//                Controller primaryTranslationalForward = Controller.proportional(0.24272093422833446);
//                Controller secondaryTranslationalForward = Controller.proportional(0.08967889957455957);
//                Controller primaryTranslationalLateral = Controller.proportional(0.3288816777498326);
//                Controller secondaryTranslationalLateral = Controller.proportional(0.12151299204828499);
//
//                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
//                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
//
//                c.coast.set(Controller.proportionalFeedforward(0.012190672787850902));
//                c.brake.set(Controller.proportionalFeedforward(0.010362071869673266));
//
//                c.headingFeedback.set(Controller.proportional(4.888003759741721));
//                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04718802021541273, 0.011056550422886387));
//
//                c.linearBrakeCoefficients.set(Matrix.diag(0.08367907486665169, 0.018703400932829052));
//                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0015711025964169948, 0.0027302381709496727));
//
//                c.maxAchievableForwardVelocity.set(82.55726519955314);
//                c.maxAchievableStrafeVelocity.set(67.41403942249028);
//                c.naturalForwardDeceleration.set(31.433927471487273);
//                c.naturalStrafeDeceleration.set(56.58232370585322);
//            }
//    );
}
