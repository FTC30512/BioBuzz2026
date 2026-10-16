package org.firstinspires.ftc.teamcode.pedro.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

@Autonomous
public class BlobDetection extends LinearOpMode{

    private Limelight3A limelight;

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight");
        limelight.start();

        telemetry.addLine("BlobDetection");
        telemetry.addLine("Limelight starting...");
        telemetry.update();

        waitForStart();while (opModeIsActive()) {

            String detectedColor = detectColor();

            telemetry.addData("Detected Color", detectedColor);

            if (detectedColor.equals("YELLOW")) {
                telemetry.addLine("POLLEN");
            }
            else if (detectedColor.equals("RED")) {
                telemetry.addLine("NECTAR");
            }
            else if (detectedColor.equals("BLUE")) {
                telemetry.addLine("NECTAR");
            }
            else {
                telemetry.addLine("NO GAME PIECE");
            }

            telemetry.update();
        }

        limelight.stop();
    }

    private String detectColor() {

        limelight.pipelineSwitch(0);
        sleep(50);

        if (hasBlob()) {
            return "YELLOW";
        }

//        limelight.pipelineSwitch(1);
//        sleep(50);
//
//        if (hasBlob()) {
//            return "RED";
//        }
//
//        limelight.pipelineSwitch(2);
//        sleep(50);

//        if (hasBlob()) {
//            return "BLUE";
//        }

        return "Other";
    }

    private boolean hasBlob() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return false;
        }

        List<LLResultTypes.ColorResult> blobs =
                result.getColorResults();

        return blobs != null && !blobs.isEmpty();
    }

}
