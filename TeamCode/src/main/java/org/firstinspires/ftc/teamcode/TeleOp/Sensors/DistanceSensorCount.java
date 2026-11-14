package org.firstinspires.ftc.teamcode.TeleOp.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "DistanceSensorCount", group = "Test")
public class DistanceSensorCount extends LinearOpMode {

    private DigitalChannel intakeSensor;

    private int ballCount = -1;

    private boolean previousDetected = false;

    @Override
    public void runOpMode() {

        intakeSensor = hardwareMap.get(DigitalChannel.class, "distanceSensor");
        intakeSensor.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addLine("Ball Counter Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean currentDetected = !intakeSensor.getState();
            
            if (currentDetected && !previousDetected) {
                ballCount++;
            }

            previousDetected = currentDetected;

            telemetry.addData("Ball Detected", currentDetected);
            telemetry.addData("Ball Count", ballCount);
            telemetry.update();
        }
    }
}