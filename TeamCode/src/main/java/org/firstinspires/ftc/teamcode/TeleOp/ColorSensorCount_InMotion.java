package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "ColorSensorCount_InMotion", group = "Test")
public class ColorSensorCount_InMotion extends LinearOpMode {

    private RevColorSensorV3 colorSensor;
    private int ballCount = 0;

    // Reuse HSV array to avoid allocations every loop
    private final float[] hsv = new float[3];

    // State tracking
    private boolean wasYellow = false;
    private final ElapsedTime debounceTimer = new ElapsedTime();

    // ===== TUNING VALUES FOR IN-MOTION DETECTION =====
    private static final float SENSOR_GAIN = 20.0f;
    private static final float YELLOW_MIN_HUE = 30f;
    private static final float YELLOW_MAX_HUE = 85f;
    private static final float MIN_SATURATION = 0.25f;
    private static final float MIN_VALUE = 0.01f;
    private static final double DEBOUNCE_MS = 40;

    public void initSensor() {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color_count");
        colorSensor.setGain(SENSOR_GAIN);
        debounceTimer.reset();
    }

    public void updateSensor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        // Scale normalized floats directly to 0-255 to prevent toColor() precision loss
        int red = (int) Math.min(255, colors.red * 255f);
        int green = (int) Math.min(255, colors.green * 255f);
        int blue = (int) Math.min(255, colors.blue * 255f);

        Color.RGBToHSV(red, green, blue, hsv);

        float hue = hsv[0];
        float saturation = hsv[1];
        float value = hsv[2];

        boolean isYellow =
                hue >= YELLOW_MIN_HUE &&
                        hue <= YELLOW_MAX_HUE &&
                        saturation >= MIN_SATURATION &&
                        value >= MIN_VALUE;

        // Count only on transition from non-yellow to yellow
        if (isYellow && !wasYellow && debounceTimer.milliseconds() > DEBOUNCE_MS) {
            ballCount++;
            debounceTimer.reset();
        }

        wasYellow = isYellow;

        telemetry.addData("Yellow", isYellow);
        telemetry.addData("Count", ballCount);
        telemetry.addData("HSV", "H: %.1f  S: %.2f  V: %.2f", hue, saturation, value);
    }

    public int getBallCount() {
        return ballCount;
    }

    public void reset() {
        ballCount = 0;
        wasYellow = false;
        debounceTimer.reset();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        
    }
}
