//Controls:
// gamepad1.b = resetYaw/resetHeading
// gamepad1.y = reverse intake
// gamepad1.dpad_up = turn off intake
// gamepad1.right_trigger = shoot
// gamepad1.left_bumper = lower/lift dolphin flap (toggle)

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Blue TeleOp", group = "Main")
public class BlueTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    // --- Motors ---
    private DcMotor leftFront, leftRear, rightFront, rightRear, intake, integration;
    private DcMotorEx shooter;
    private CRServo leftCR, rightCR;
    private Servo leftDolphin, rightDolphin;
    private IMU imu;
    private GoBildaPinpointDriver pinpoint;
    Limelight3A limelight;
    private LLResult llResult;
    private boolean isTurning = false;
    private double actYaw = 0;
    private double maxTurnSpeed = 0.5;
    // --- Helper Classes ---
    private Movement movement;
    private double Kp = 255.0, Ki = 0.0, Kd = 0.0, Kf = 11.62;
    private boolean is_lowered = false;
    private double tag_offset = 0;

    // 1. ADD THIS TRACKING VARIABLE HERE:
    private boolean lastLeftBumper = false;

    @Override
    public void runOpMode() {

        // --- Initialize hardware ---
        initHardware();
        liftDolphin();
        // --- Initialize helpers ---
        movement = new Movement(leftFront, leftRear, rightFront, rightRear, intake, gamepad1, pinpoint, telemetry);

        telemetry.addLine("Initialized. Press PLAY to start.");
        telemetry.update();
        gamepad1.rumble(500);

        waitForStart();

        // --- Main loop ---
        while (opModeIsActive()) {

            shooter.setVelocity(1350);
            pinpoint.update ();

            if (gamepad1.b) {
                resetHeading ();
            }
//             --- Intake ---
            if (gamepad1.y)
            {
                intake.setPower ( -1 );
            }else{
                intake.setPower(1);
            }

            if (gamepad1.dpad_up){
                intake.setPower(0);
            }

            // --- Driving ---
            movement.drive( pinpoint.getHeading (AngleUnit.RADIANS) );

            if (gamepad1.right_trigger > 0.5){
                isTurning = true;
                shoot();
            }
            else{
                leftCR.setPower(0);
                rightCR.setPower(0);
                integration.setPower(0);
            }

            // 2. REPLACE YOUR OLD left_bumper CHECK WITH THIS:
            boolean currentLeftBumper = gamepad1.left_bumper;
            if (currentLeftBumper && !lastLeftBumper) {
                if (is_lowered) {
                    liftDolphin();
                } else {
                    lowerDolphin();
                }
            }
            lastLeftBumper = currentLeftBumper; // Save state for the next loop iteration
            double currentHeading = getHeading();
            limelight.updateRobotOrientation(getRawHeading());
            llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()){
                telemetry.addData("Tx", llResult.getTx());
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Ta", llResult.getTa());
                telemetry.addLine("Current Heading angle" + getHeading());
                telemetry.addLine("pinpoint angle" + pinpoint.getHeading(AngleUnit.DEGREES));

                telemetry.update();
            }
            // --- Telemetry ---
//            telemetry.addData("Actual Yaw: ", actYaw);
//            telemetry.addData("Tx: ", llResult.getTx());
//            telemetry.addData("Valid: ", llResult.isValid());
//            telemetry.addData("Null: ", llResult!=null);
//            telemetry.addData("is turning: ", isTurning);
//            telemetry.addData ( "pinpoint heading: ", pinpoint.getHeading ( AngleUnit.DEGREES ) );

            telemetry.update ();
//            telemetry.addData("Pinpoint Heading: ", pinpoint.getHeading(AngleUnit.DEGREES));
//            telemetry.addData("imu heading: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//            telemetry.update();
        }
    }

    // --- Hardware initialization method ---
    private void initHardware() {
        // Motors
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        intake = hardwareMap.dcMotor.get("Intake");
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");

        integration = hardwareMap.dcMotor.get("Integration");
        leftCR = hardwareMap.get(CRServo.class, "leftCR");
        rightCR = hardwareMap.get(CRServo.class, "rightCR");

        leftDolphin = hardwareMap.get(Servo.class, "leftDolphin");
        rightDolphin = hardwareMap.get(Servo.class, "rightDolphin");
        leftDolphin.setDirection(Servo.Direction.REVERSE);


        shooter.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Sensors
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets ( -7.0, -5.625, DistanceUnit.INCH );
        pinpoint.setEncoderDirections ( GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD );
        pinpoint.setEncoderResolution ( GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD );
        pinpoint.resetPosAndIMU ();

        // Sensors

//        imu = hardwareMap.get(IMU.class, "imu");
//
//        IMU.Parameters parameters = new IMU.Parameters(
//                new RevHubOrientationOnRobot(
//                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
//                )
//        );
//        imu.initialize(parameters);

        for (DcMotor m : new DcMotor[]{leftFront, leftRear, rightFront, rightRear}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        limelight = hardwareMap.get ( Limelight3A.class, "limelight" );
        limelight.pipelineSwitch(1);
        limelight.start();

        runtime.reset();
    }
    public void lowerDolphin(){
        leftDolphin.setPosition(0.535);
        rightDolphin.setPosition(0.55);
        is_lowered = true;
    }
    public void liftDolphin(){
        leftDolphin.setPosition(0.1);
        rightDolphin.setPosition(0.1);
        is_lowered = false;
    }
    // --- Shooting method ---
    public void shoot() {
        if (llResult != null && llResult.isValid()) {
            telemetry.addLine("April Tag Detected");
            pid_turn_by_gyro(llResult.getTx() + tag_offset);
        }
        else {
            telemetry.addLine("April Tag NOT Detected!!!");
            isTurning = false;
        }

        if(!isTurning) {
            leftCR.setPower(-1);
            rightCR.setPower(1);
            integration.setPower(1);
        }
        else{
            leftCR.setPower(0);
            rightCR.setPower(0);
            integration.setPower(0);
        }
    }
    private double getHeading() {
        double angle = getRawHeading();
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    private double getRawHeading() {
        return pinpoint.getHeading ( AngleUnit.DEGREES );
    }

    public void pid_turn_by_gyro(double targetYaw){

        if (isTurning) {
            double error = -targetYaw;

            if (Math.abs(error) > 1) {
                double kp = 0.02;
                double power = error * kp;

                // Bound the power limits smoothly
                if (power > 0) {
                    power = Math.min(Math.max(power, 0.125), maxTurnSpeed);
                } else {
                    power = Math.min(Math.max(power, -maxTurnSpeed), -0.125);
                }
//
//                telemetry.addData("Tx", llResult.getTx());
//                telemetry.addData("Ty", llResult.getTy());
//                telemetry.addData("Ta", llResult.getTa());
//                telemetry.addData("Target Angle", actYaw);
//                telemetry.addData("Actual Power", power);

                // Uncomment this to make the wheels physically move:
                setDrivePower(-power, -power, power, power);
            } else {
                // Target reached
                setDrivePower(0, 0, 0, 0);
                isTurning = false;
            }
        }

    }
    private void setDrivePower(double lfPower, double lrPower, double rfPower, double rrPower)
    {
        leftFront.setPower(lfPower);
        leftRear.setPower(lrPower);
        rightFront.setPower(rfPower);
        rightRear.setPower(rrPower);
    }
    public void resetHeading() {
        pinpoint.resetPosAndIMU ();
        //imu.resetYaw();
    }
}