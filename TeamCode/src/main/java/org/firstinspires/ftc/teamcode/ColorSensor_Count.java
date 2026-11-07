package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensor_Count {
    NormalizedColorSensor colorSensor;

    public enum detectedColor{
        YELLOW,
        UNKOWN
    }

    public void init(HardwareMap hwMap) {

        colorSensor = hwMap.get(NormalizedColorSensor.class, "sensor_color_count");
    }

    public detectedColor getDetectedColor (Telemetry telemetry) {


    }



}
