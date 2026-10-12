package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "Limelight Pipeline Checker", group = "Sensor")
public class LimelightAutonomous extends LinearOpMode {
    private Limelight3A limelight;

    // Set to true to target Pipeline 2, or false to target Pipeline 1
    private boolean targetRedCell = true;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Limelight hardware (assuming it's configured in your hardware map)
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // ==========================================
            // STEP 1: Run and check Pipeline 0
            // ==========================================
            limelight.pipelineSwitch(0);
            telemetry.addData("Pipeline", "0 (Initial Detection)");
            telemetry.update();

            sleep(1000); // Give the camera time to process

            LLResult result0 = limelight.getLatestResult();
            if (result0 != null && result0.isValid()) {
                telemetry.addData("Pipeline 0 Target", "Found (ID: " + result0.getFiducialResults() + ")");
            } else {
                telemetry.addData("Pipeline 0 Target", "Not Found");
            }
            telemetry.update();
            sleep(1000);

            // ==========================================
            // STEP 2: Switch to Pipeline 1 or 2 based on selection
            // ==========================================
            int targetPipeline = targetRedCell ? 2 : 1;
            limelight.pipelineSwitch(targetPipeline);

            telemetry.addData("Pipeline", targetPipeline + " (" + (targetRedCell ? "RedCell" : "BlueCell") + ")");
            telemetry.update();

            sleep(1000); // Allow camera to stabilize

            // Continuous check loop
            while (opModeIsActive()) {
                LLResult resultTarget = limelight.getLatestResult();

                if (resultTarget != null && resultTarget.isValid()) {
                    double tx = resultTarget.getTx();
                    double ty = resultTarget.getTy();
                    List<LLResultTypes.FiducialResult> tagId = resultTarget.getFiducialResults();

                    telemetry.addData("Active Pipeline", targetPipeline);
                    telemetry.addData("Tag ID", tagId);
                    telemetry.addData("Tx (X Offset)", tx);
                    telemetry.addData("Ty (Y Offset)", ty);
                } else {
                    telemetry.addData("Active Pipeline", targetPipeline);
                    telemetry.addData("Status", "Searching for target...");
                }
                telemetry.update();
                sleep(20);
            }
        }
    }
}