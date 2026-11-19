package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Movement {

    private final DcMotor lf, lr, rf, rr;
    private final Gamepad gamepad;
    private final Telemetry telemetry;

    public Movement(DcMotor lf, DcMotor lr, DcMotor rf, DcMotor rr, Gamepad gamepad, Telemetry telemetry) {
        this.lf = lf;
        this.lr = lr;
        this.rf = rf;
        this.rr = rr;
        this.gamepad = gamepad;
        this.telemetry = telemetry;
    }

    public void drive(double botHeading) {
        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x;

        // Field-centric rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1; // strafe compensation

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        lf.setPower(frontLeftPower);
        lr.setPower(backLeftPower);
        rf.setPower(frontRightPower);
        rr.setPower(backRightPower);
    }
}