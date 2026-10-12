package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "Limelight AprilTag Distance", group = "Sensor")
public class LimelightDistanceDetector extends LinearOpMode {
    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        // Initialize the Limelight from the hardware map
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Switch to your AprilTag pipeline index (e.g., pipeline 1 or 2)
        limelight.pipelineSwitch(1);
        limelight.start();

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get the latest result from the Limelight
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // Check if any AprilTags (fiducials) are detected
                for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                    int tagId = fiducial.getFiducialId();

                    // Get the 3D position and orientation of the tag relative to the camera lens
                    Pose3D targetPose = fiducial.getTargetPoseCameraSpace();

                    if (targetPose != null) {
                        // The Z coordinate represents the straight-line distance forward from the lens (in meters)
                        double distanceMeters = targetPose.getPosition().z;

                        // Convert meters to inches for the Driver Hub display
                        double distanceInches = distanceMeters * 39.3701;

                        // Display the detected tag and its calculated distance
                        telemetry.addData("Detected Tag ID", tagId);
                        telemetry.addData("Distance", "%.2f inches (%.2f meters)", distanceInches, distanceMeters);
                    }
                }
            } else {
                telemetry.addData("Status", "No AprilTag detected");
            }

            telemetry.update();
            sleep(20); // Small pause to prevent CPU overuse
        }
    }
}
