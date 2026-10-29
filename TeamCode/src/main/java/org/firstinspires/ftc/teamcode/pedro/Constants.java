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
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.253403284776876);
                Controller secondaryTranslationalForward = Controller.proportional(0.09362574266458204);
                Controller primaryTranslationalLateral = Controller.proportional(0.286512140047428);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1058585800021447);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.016177233893588834));
                c.brake.set(Controller.proportionalFeedforward(0.013750648809550509));

                c.headingFeedback.set(Controller.proportional(4.359386458929825));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04301682754943221, 0.005494985019496217));

                c.linearBrakeCoefficients.set(Matrix.diag(0.08376067529703282, 0.06955930995992894));
                c.quadraticBrakeCoefficients.set(Matrix.diag(9.826909827781915E-4, 0.0011949808379995324));

                c.maxAchievableForwardVelocity.set(65.04836663212309);
                c.maxAchievableStrafeVelocity.set(53.58477552002064);
                c.naturalForwardDeceleration.set(32.10030327930075);
                c.naturalStrafeDeceleration.set(50.28396870278229);
            }
    );
}