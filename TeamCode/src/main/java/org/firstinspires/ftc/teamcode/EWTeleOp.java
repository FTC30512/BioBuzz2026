package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

@TeleOp(name = "EW TeleOp", group = "Main")
public class EWTeleOp extends LinearOpMode {

    private LLResult llResult;
    private ElapsedTime runtime = new ElapsedTime();

    private Limelight3A limelight;

    // --- Motors ---
    private DcMotor leftFront, leftRear, rightFront, rightRear, intake;
    private DcMotorEx shooter;

    // --- Sensors ---
    private IMU imu;

    // --- Helper Classes ---
    private Movement movement;


    private int IdNum = 25;

    private double offset;

    private int shooterSpeedMin = 2200, shooterSpeedMax = 2475;
    private double shooterSpeed;

    //private double Kp = 255.0, Ki = 0.0, Kd = 0.0, Kf = 11.62;
    @Override
    public void runOpMode() {

        // --- Initialize hardware ---
        initHardware();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        limelight.start();


        // --- Initialize helpers ---
        movement = new Movement(leftFront, leftRear, rightFront, rightRear, intake, gamepad1, imu);

        telemetry.addLine("Initialized. Press PLAY to start.");
        telemetry.update();
        gamepad1.rumble(500);

        waitForStart();

        limelight.start();

        // --- Main loop ---
        while (opModeIsActive()) {

            if (runtime.seconds() > 105){
                gamepad1.rumble(15000);
            }

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
            llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()){
                Pose3D botPose = llResult.getBotpose();
//                telemetry.addData("Tx", llResult.getTx());
//                telemetry.addData("Ty", llResult.getTy());
//                telemetry.addData("Ta", llResult.getTa());
//                telemetry.update();
            }

            if (gamepad1.left_bumper){
                intake.setPower(-1);
            }else {
                intake.setPower(1);
            }

            if (gamepad1.dpad_up){
                intake.setPower(0);
            }

            // --- Driving ---
            movement.drive();

            // --- Intake ---
            intake.setPower(1);

            if (gamepad1.right_trigger > 0.5){
                shoot();
                telemetry.addLine("Shooting");
            }
            if (llResult != null && llResult.isValid()){
                if (llResult.getTa() > 0.35) {
                    shooterSpeed = (-275 * llResult.getTa()) + 2557.5;
                }else{
                    shooterSpeed = 2900;
                }
            }

            if (llResult != null && llResult.isValid() && !llResult.getFiducialResults().isEmpty()) {
                IdNum = llResult.getFiducialResults().get(0).getFiducialId();
                telemetry.addData("AprilTag ID", IdNum);
            } else {
                telemetry.addLine("No AprilTag detected");
            }

            if (IdNum == 24){
                offset = 2.5 ;
            } else if (IdNum == 20) {
                offset = 1;
            }

            shooter.setVelocity(shooterSpeed);

            if (gamepad1.x) {
                shoot();
                telemetry.addLine("Shooting");
            }

            // --- Telemetry ---
//            telemetry.addData("Shooter Servo Position", shooterServo.getPosition());
//            telemetry.addData("Shooter Power", shooter.getPower());
//            telemetry.update();
            telemetry.addData("Lx", gamepad1.left_stick_x);
            telemetry.addData("Ly", gamepad1.left_stick_y);
            telemetry.addData("ShooterVelocity", shooter.getVelocity());
            telemetry.update();
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

        //shooter.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // Sensors
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );
        imu.initialize(parameters);

        AprilTagLibrary aprilTagLibrary = new AprilTagLibrary.Builder()
                .build();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{leftFront, leftRear, rightFront, rightRear}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        runtime.reset();
    }

    // --- Shooting method ---
    public void shoot() {

    }

    private double getHeading() {
        double angle = getRawHeading();
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    private double getRawHeading() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.DEGREES);
    }

    public void pid_turn_by_gyro(double targetYaw, double speed){
        targetYaw=-targetYaw;
        double currentYaw = getHeading();
        double error;
        double actYaw = getHeading() + targetYaw;
        double kp = 0.02;
        while(Math.abs(actYaw - getHeading()) > 0.5){

            error = kp * (actYaw - getHeading());
            double power;
            if(error > 0)
                power = Math.min(Math.max(error, 0.15), speed);
            else
                power = Math.min(Math.max(error, -speed), -0.15);

            telemetry.addLine("Current Heading angle" + getHeading());
            telemetry.addLine( "Target Angle" + targetYaw);
            telemetry.addLine("Actual Target Yaw" + actYaw);
            telemetry.addLine("Actual Power " + power);
            telemetry.update();
            setDrivePower(-power, -power, power, power );
        }
        setDrivePower(0,0,0,0 );

    }

    private void setDrivePower(double lfPower, double lrPower, double rfPower, double rrPower)
    {
        leftFront.setPower(lfPower);
        leftRear.setPower(lrPower);
        rightFront.setPower(rfPower);
        rightRear.setPower(rrPower);
    }
}

