package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class RobotFunctions {

    // Motors
    public DcMotor intake;
    public DcMotor integration;
    public DcMotorEx shooter;

    // Servos
    public CRServo leftCR;
    public CRServo rightCR;

    public Servo leftDolphin;
    public Servo rightDolphin;

    // Sensors
    public GoBildaPinpointDriver pinpoint;
    public Limelight3A limelight;

    // States
    public boolean isLowered = false;

    // Shooter PIDF
    private final double Kp = 255.0;
    private final double Ki = 0.0;
    private final double Kd = 0.0;
    private final double Kf = 11.62;

    public RobotFunctions(HardwareMap hardwareMap) {

        // Motors
        intake = hardwareMap.get(DcMotor.class, "Intake");
        integration = hardwareMap.get(DcMotor.class, "Integration");
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");

        // CR Servos
        leftCR = hardwareMap.get(CRServo.class, "leftCR");
        rightCR = hardwareMap.get(CRServo.class, "rightCR");

        // Servos
        leftDolphin = hardwareMap.get(Servo.class, "leftDolphin");
        rightDolphin = hardwareMap.get(Servo.class, "rightDolphin");

        // Sensors
        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        // Directions
        leftDolphin.setDirection(
                Servo.Direction.REVERSE
        );

        intake.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        shooter.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        // Shooter PIDF
        shooter.setVelocityPIDFCoefficients(
                Kp,
                Ki,
                Kd,
                Kf
        );

        // Pinpoint Setup
        pinpoint.setOffsets(
                -7.0,
                -5.625,
                DistanceUnit.INCH
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.resetPosAndIMU();

        // Limelight
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    // -------- DOLPHIN --------

    public void lowerDolphin() {
        leftDolphin.setPosition(0.535);
        rightDolphin.setPosition(0.55);
        isLowered = true;
    }

    public void liftDolphin() {
        leftDolphin.setPosition(0.1);
        rightDolphin.setPosition(0.1);
        isLowered = false;
    }

    // -------- SHOOTER --------

    public void shoot() {
        leftCR.setPower(-1);
        rightCR.setPower(1);
        integration.setPower(1);
    }

    public void stopShooting() {
        leftCR.setPower(0);
        rightCR.setPower(0);
        integration.setPower(0);
    }

    public void setShooterVelocity(double velocity) {
        shooter.setVelocity(velocity);
    }

    // -------- INTAKE --------

    public void intakeForward() {
        intake.setPower(0.85);
    }

    public void intakeReverse() {
        intake.setPower(-1);
    }

    public void intakeStop() {
        intake.setPower(0);
    }

    // -------- HEADING --------

    public void resetHeading() {
        pinpoint.resetPosAndIMU();
    }

    public void resetToCorner() {
        pinpoint.setPosition(
                new Pose2D(
                        DistanceUnit.INCH,
                        9.0,
                        8.5,
                        AngleUnit.DEGREES,
                        0.0
                )
        );
    }

    public double getRawHeading() {
        return pinpoint.getHeading(
                AngleUnit.DEGREES
        );
    }

    public double getHeading() {

        double angle = getRawHeading();

        if (angle > 180) {
            angle -= 360;
        }

        if (angle < -180) {
            angle += 360;
        }

        return angle;
    }

    // -------- LIMELIGHT --------

    public void updateLimelightHeading() {
        limelight.updateRobotOrientation(
                getRawHeading()
        );
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }
}