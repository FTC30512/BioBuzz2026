package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "DistanceSensorCount", group = "Test")
public class DistanceSensorCount extends LinearOpMode {

    private DigitalChannel intakeSensor;
    private DigitalChannel ledPin1;
    private DigitalChannel ledPin2;

    private int ballCount = 0;
    private boolean previousSensorState = false;

    @Override
    public void runOpMode() {
        intakeSensor = hardwareMap.get(DigitalChannel.class, "distanceSensor");
        intakeSensor.setMode(DigitalChannel.Mode.INPUT);

        ledPin1 = hardwareMap.get(DigitalChannel.class, "led1");
        ledPin1.setMode(DigitalChannel.Mode.OUTPUT);

        ledPin2 = hardwareMap.get(DigitalChannel.class, "led2");
        ledPin2.setMode(DigitalChannel.Mode.OUTPUT);

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean rawSensorState = intakeSensor.getState();
            if (previousSensorState && !rawSensorState) {
                ballCount++;
            }
            previousSensorState = rawSensorState;

            if (ballCount < 4) {
                // Off
                ledPin1.setState(true);
                ledPin2.setState(true);
            } else if (ballCount == 4) {
                // Yellow
                ledPin1.setState(true);
                ledPin2.setState(false);
            } else {
                // Red
                ledPin1.setState(false);
                ledPin2.setState(true);
            }

            telemetry.addData("Ball Count", ballCount);
            telemetry.addData("LED Status", ballCount < 4 ? "Yellow" : (ballCount == 4 ? "Green" : "Red"));

            telemetry.update();
        }
    }
}