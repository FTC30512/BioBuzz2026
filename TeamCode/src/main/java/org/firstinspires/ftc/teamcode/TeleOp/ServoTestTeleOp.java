package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTestTeleOp extends OpMode {
    ServoTest shooter1 = new ServoTest();
    ServoTest shooter2 = new ServoTest();
//    ServoTest dolphinArm = new ServoTest();

    @Override
    public void init() {
        shooter1.init(hardwareMap);
        shooter2.init(hardwareMap);
//        dolphinArm.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper) {
            shooter1.setServoRot(1);
        } else {
            shooter1.setServoRot(0);
        }
        if(gamepad1.right_bumper) {
            shooter2.setServoRot(-1);
        } else {
            shooter2.setServoRot(0);
        }
    }
}
