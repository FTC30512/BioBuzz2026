package org.firstinspires.ftc.teamcode.TestMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class TestMotorANDServo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double Kp = 255.0, Ki = 0.0, Kd = 0.0, Kf = 11.62;

        DcMotor intake = hardwareMap.get(DcMotor.class, "Intake");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        DcMotor integration = hardwareMap.get(DcMotor.class, "Integration");
        CRServo leftCR = hardwareMap.get(CRServo.class, "leftCR");
        CRServo rightCR = hardwareMap.get(CRServo.class, "rightCR");
        shooter.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf);

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            shooter.setVelocity(1400);
            intake.setPower(-1);

            if (gamepad1.right_trigger>0.5) {
                leftCR.setPower(-1);
                rightCR.setPower(1);
                integration.setPower(1);

            } else{
                leftCR.setPower(0);
                rightCR.setPower(0);
                integration.setPower(0);
            }
        }
    }
}
