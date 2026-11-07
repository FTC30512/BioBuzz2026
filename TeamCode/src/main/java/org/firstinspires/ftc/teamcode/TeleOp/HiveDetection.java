package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Hive Detection", group = "Test")
public class HiveDetection extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(1);
        limelight.start();

        telemetry.addLine("Ready! Press PLAY to start testing.");
        telemetry.update();

        waitForStart();
        resetRuntime();
        while (opModeIsActive()) {
            LLResult llResult = limelight.getLatestResult();
            telemetry.addLine("Runtime: " + getRuntime());
            if (llResult != null && llResult.isValid()) {
                if (!llResult.getFiducialResults().isEmpty()) {
                    telemetry.addLine("=== TAG DETECTED ===");
                    telemetry.addData("Tags Visible", llResult.getFiducialResults().get(0).getFiducialId());

                }
            } else {
                if (getRuntime() >= 5) {
                    telemetry.addLine("No Hive Tip");
                } else {
                    telemetry.addLine("Waiting for a hive tipe");
                }
            }
            telemetry.update();
        }
        limelight.stop();
    }
}
