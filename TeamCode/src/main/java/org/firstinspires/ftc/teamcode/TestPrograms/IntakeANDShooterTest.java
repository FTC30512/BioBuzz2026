package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Intake and Shooters", group = "Test")
public class IntakeANDShooterTest extends LinearOpMode {
    private DcMotor intake;
    private DcMotorEx shooterRight, shooterLeft;

    @Override
    public void runOpMode() {

        initHardware();

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                intake.setPower(0);
            } else if (gamepad1.y) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(1);
            }

            if (gamepad1.right_trigger > 0.5) {
                shooterRight.setPower(-0.8);
            } else {
                shooterRight.setPower(0);
            }

            if (gamepad1.left_trigger > 0.5) {
                shooterLeft.setPower(-0.8);
            } else {
                shooterLeft.setPower(0);
            }

            telemetry.update();
        }
    }

    private void initHardware() {

        intake = hardwareMap.dcMotor.get("intake");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}