package org.firstinspires.ftc.teamcode.TestMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class TestMotorANDServo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor intake = hardwareMap.get(DcMotor.class, "Intake");
        DcMotor shooter = hardwareMap.get(DcMotor.class, "Shooter");
        CRServo leftCR = hardwareMap.get(CRServo.class, "leftCR");
        CRServo rightCR = hardwareMap.get(CRServo.class, "rightCR");

        waitForStart();

        while (opModeIsActive()) {
            shooter.setPower(-0.7);
            intake.setPower(-1);

            if (gamepad1.right_trigger>0.5) {
                leftCR.setPower(-1);
                rightCR.setPower(1);
            } else{
                leftCR.setPower(1);
                rightCR.setPower(-1);
            }
        }
    }
}
