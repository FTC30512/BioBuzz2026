package org.firstinspires.ftc.teamcode.pedro;

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
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftRear");
        c.backRightName.set("rightRear");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-7.0);
        c.yPodOffset.set(-5.625);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2510977962167086);
                Controller secondaryTranslationalForward = Controller.proportional(0.09277392624538908);
                Controller primaryTranslationalLateral = Controller.proportional(0.3460815590324366);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1278678885929149);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.016234432922904104));
                c.brake.set(Controller.proportionalFeedforward(0.013799267984468488));

                c.headingFeedback.set(Controller.proportional(4.1067515097087455));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.045630285495884396, 0.0058373097107914775));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07791355671628335, 0.06340034117626452));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001222585423377711, 0.0011356067925699168));

                c.maxAchievableForwardVelocity.set(62.75388101787602);
                c.maxAchievableStrafeVelocity.set(52.04136303257392);
                c.naturalForwardDeceleration.set(30.21110024213363);
                c.naturalStrafeDeceleration.set(55.95401130225255);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}



//OLD ROBOT CONSTANTS (DECODE ROBOT)



//package org.firstinspires.ftc.teamcode.pedro;
//
//import com.pedropathing.algorithm.Foresight;
//import com.pedropathing.algorithm.ForesightConfig;
//import com.pedropathing.controllers.Controller;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.math.Matrix;
//import com.pedropathing.math.Vector2D;
//import com.pedropathing.revhub.drivetrains.Mecanum;
//import com.pedropathing.revhub.drivetrains.MecanumConfig;
//import com.pedropathing.revhub.localizers.PinpointConfig;
//import com.pedropathing.revhub.localizers.PinpointLocalizer;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//public class Constants {
//    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
//        c.frontLeftName.set("leftFront");
//        c.frontRightName.set("rightFront");
//        c.backLeftName.set("leftRear");
//        c.backRightName.set("rightRear");
//        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
//        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
//        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
//        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
//    });
//    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
//        c.name.set("pinpoint");
//        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        c.xPodOffset.set(-5.25);
//        c.yPodOffset.set(5.25);
//        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        c.globalDistanceUnit.set(DistanceUnit.INCH);
//        c.offsetUnits.set(DistanceUnit.INCH);
//    });
//    public static ForesightConfig foresightConfig = new ForesightConfig(
//            c -> {
//                Controller primaryTranslationalForward = Controller.proportional(0.3427794610828041);
//                Controller secondaryTranslationalForward = Controller.proportional(0.12664785163420791);
//                Controller primaryTranslationalLateral = Controller.proportional(0.4871207443974941);
//                Controller secondaryTranslationalLateral = Controller.proportional(0.17997809894886274);
//
//                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
//                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
//
//                c.coast.set(Controller.proportionalFeedforward(0.018752680608852927));
//                c.brake.set(Controller.proportionalFeedforward(0.015939778517524986));
//
//                c.headingFeedback.set(Controller.proportional(4.330563470329443));
//                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04620651486019679, 0.008125626678465298));
//
//                c.linearBrakeCoefficients.set(Matrix.diag(0.0801411321102565, 0.10767291417380488));
//                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001226993297621214, -4.5186203368677633E-4));
//
//                c.maxAchievableForwardVelocity.set(57.8090361279374);
//                c.maxAchievableStrafeVelocity.set(47.19379692992167);
//                c.naturalForwardDeceleration.set(33.51877640021743);
//                c.naturalStrafeDeceleration.set(61.66581987287369);
//            }
//    );
//    public static Follower create(HardwareMap h) {
//        return new Follower(
//                new PinpointLocalizer(h, localizerConfig),
//                new Mecanum(h, drivetrainConfig),
//                new Foresight(foresightConfig)
//        );
//    }
//}