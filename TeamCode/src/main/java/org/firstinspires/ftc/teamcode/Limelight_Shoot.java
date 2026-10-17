package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

@Autonomous(name = "Limelight Simple Auto", group = "Sensor")
public class Limelight_Shoot extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor frontLeft, backLeft, frontRight, backRight;

    // true = Red side, false = Blue side
    private boolean targetRedCell = true;

    private static final int RED_PIPELINE = 2;
    private static final int BLUE_PIPELINE = 1;
    private static final double TURN_SPEED = 0.15;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");


        waitForStart();
        limelight.start();

        int ownPipeline = targetRedCell ? RED_PIPELINE : BLUE_PIPELINE;
        int otherPipeline = targetRedCell ? BLUE_PIPELINE : RED_PIPELINE;

        // 1. Look for our own side's tag
        limelight.pipelineSwitch(ownPipeline);
        sleep(1000);

        // 2. Not found? Strafe to the other side and look for that side's tag
        if (getTag() == null) {
            double direction = targetRedCell ? 1 : -1;   // flip if it goes the wrong way
            sleep(1200);

            limelight.pipelineSwitch(otherPipeline);
            sleep(1000);
        }

        // 3. Turn until the tag is centered
        while (opModeIsActive()) {
            LLResultTypes.FiducialResult tag = getTag();

            if (tag == null) {
                telemetry.addData("Status", "No tag found");
            } else if (Math.abs(tag.getTargetXDegrees()) < 1.0) {
                telemetry.addData("Status", "Centered - firing!");
                telemetry.update();
                break;
            } else if (tag.getTargetXDegrees() > 0) {
                telemetry.addData("Status", "Turning right");
            } else {
                telemetry.addData("Status", "Turning left");
            }
            telemetry.update();
        }

        // 4. Shoot (display only - no shooter code yet)
        if (opModeIsActive()) {
            telemetry.clearAll();
            telemetry.addData("Action", "SHOOT");
            telemetry.update();
            sleep(3000);   // keeps SHOOT on the screen for 3 seconds
        }
        limelight.stop();
    }

    // Returns the first AprilTag the camera sees, or null if none
    private LLResultTypes.FiducialResult getTag() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return null;
        }
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            return null;
        }
        return tags.get(0);
    }

    }