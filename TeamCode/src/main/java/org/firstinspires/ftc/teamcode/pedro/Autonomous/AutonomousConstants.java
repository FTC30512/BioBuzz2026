package org.firstinspires.ftc.teamcode.pedro.Autonomous;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

public class AutonomousConstants {

    public enum PathState {
        START,
        SHOOT1,
        BLUE_GARDEN_START,
        BLUE_GARDEN_END,
        SHOOT2,
        BLUE_FLOWER_1_START,
        BLUE_FLOWER_1_END,
        BLUE_FLOWER_2_START,
        BLUE_FLOWER_2_END,
        HOME,
        STOP
    }

    // Robot Constants & Timings
    public static final double SHOOTER_VELOCITY = 1350;
    public static final double COLLECT_SPEED = 0.65;
    public static final long SHOOT_TIME_MS = 2000;
    public static final long COLLECT_FLOWER_TIME_MS = 3000;
    public static final long LOWER_DOLPHIN_TIME_MS = 1000;

    // PIDF Coefficients
    public static final double Kp = 255.0;
    public static final double Ki = 0.0;
    public static final double Kd = 0.0;
    public static final double Kf = 11.62;

    private static final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses from Blue1 Routine
    public static final Pose START =
            poseFactory.of(83, 132.5, 270);

    public static final Pose SHOOT1 =
            poseFactory.of(114, 123, 235);

    public static final Pose BLUE_GARDEN_START =
            poseFactory.of(120, 131.5, 0);

    public static final Pose BLUE_GARDEN_END =
            poseFactory.of(131, 131.5, 0);

    public static final Pose BLUE_GARDEN_END_SHOOT1_CP =
            poseFactory.of(131, 123, 0);

    // Shoot Other Side Poses
    public static final Pose SHOOT2 =
            poseFactory.of(114, 18, 125);

    public static final Pose BLUE_FLOWER_1_START =
            poseFactory.of(94.5, 22.5, 270);

    public static final Pose BLUE_FLOWER_1_END =
            poseFactory.of(94.5, 12.5, 270);

    public static final Pose HOME =
            poseFactory.of(131, 46, 180);

    public static final Pose SHOOT2_HOME_CP =
            poseFactory.of(114, 44, 0);

    // Shoot Same Side Poses
    public static final Pose BLUE_FLOWER_2_START =
            poseFactory.of(120, 94.5, 0);

    public static final Pose BLUE_FLOWER_2_END =
            poseFactory.of(127, 94.5, 0);
}