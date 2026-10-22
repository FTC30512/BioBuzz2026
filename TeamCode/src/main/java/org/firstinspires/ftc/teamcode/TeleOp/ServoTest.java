package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class ServoTest {
    private CRServo shooter1;
    private CRServo shooter2;
//    private Servo dolphinArm;
    public void init(HardwareMap hwMap) {
        shooter1 = hwMap.get(CRServo.class, "shooter1");
        shooter2 = hwMap.get(CRServo.class, "shooter2");
 //       dolphinArm = hwMap.get(Servo.class, "dolphinArm");
    }

    public void setServoRot(double power) {
        shooter1.setPower(power);
        shooter2.setPower(power);
    }
//    public void setServePos(double angle) {
//               dolphinArm.setPosition(angle);
//    }
}
