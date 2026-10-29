package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "EW TeleOp", group = "Main")
public class EWTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    // --- Motors ---
    private DcMotor leftFront, leftRear, rightFront, rightRear, intake, integration;
    private DcMotorEx shooter;
    private CRServo leftCR, rightCR;
    private Servo leftDolphin, rightDolphin;

    // --- Sensors ---
    private GoBildaPinpointDriver pinpoint;
    // --- Helper Classes ---
    private Movement movement;
    private double Kp = 255.0, Ki = 0.0, Kd = 0.0, Kf = 11.62;
    @Override
    public void runOpMode() {

        // --- Initialize hardware ---
        initHardware();

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
            intake.setPower(1);

            if (gamepad1.dpad_up){
                intake.setPower(0);
            }

            // --- Driving ---
            movement.drive();

            if (gamepad1.right_trigger > 0.5){
                shoot();
                //telemetry.addLine("Shooting");
            }
            else{
                leftCR.setPower(0);
                rightCR.setPower(0);
                integration.setPower(0);
            }
            if (gamepad1.right_bumper) {
                lowerDolphin();
            }

            if (gamepad1.left_bumper){
                liftDolphin();
            }

            // --- Telemetry ---
//            telemetry.addData("Shooter Servo Position", shooterServo.getPosition());
//            telemetry.addData("Shooter Power", shooter.getPower());
//            telemetry.update();
            //telemetry.addData("ShooterVelocity", shooter.getVelocity());
            telemetry.addData ( "pinpoint heading: ", pinpoint.getHeading ( AngleUnit.DEGREES ) );
            telemetry.update ();
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

        for (DcMotor m : new DcMotor[]{leftFront, leftRear, rightFront, rightRear}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        runtime.reset();
    }
    public void lowerDolphin(){
        leftDolphin.setPosition(0.535);
        rightDolphin.setPosition(0.8);
    }
    public void liftDolphin(){
        leftDolphin.setPosition(0.15);
        rightDolphin.setPosition(0.25);
    }
    // --- Shooting method ---
    public void shoot() {
        leftCR.setPower(-1);
        rightCR.setPower(1);
        integration.setPower(1);
    }

    public void resetHeading() {
        pinpoint.resetPosAndIMU ();
    }
}

//test