package org.firstinspires.ftc.teamcode.pedro.Autonomous;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

@Autonomous(name = "AprilTag 2-Routine Auto", group = "Autonomous")
public class RunAuto extends LinearOpMode {

    private Follower follower;
    private Limelight3A limelight;
    private LLResult llResult;

    private DcMotor intake, integration;
    private DcMotorEx shooter;
    private CRServo leftCR, rightCR;
    private Servo leftDolphin, rightDolphin;

    private static final double SHOOTER_VELOCITY = 1350;
    private final double Kp = 255.0, Ki = 0.0, Kd = 0.0, Kf = 11.62;
    private static final long SHOOT_TIME_MS = 3000;
    private static final long COLLECT_FLOWER_TIME_MS = 3000;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // position 1 (Tag ID = 1)
    private final Pose start1                  = poseFactory.of(85, 132, 270);
    private final Pose shoot1                  = poseFactory.of(85, 123, 270);
    private final Pose collectGardenPart1      = poseFactory.of(119.3584, 131, 0);
    private final Pose collectGardenPart1Ctrl  = poseFactory.of(98.2245, 124.082, 0);
    private final Pose collectGardenPart2      = poseFactory.of(132, 131, 0);
    private final Pose shoot2                  = poseFactory.of(85, 123, 270);
    private final Pose shoot2Ctrl              = poseFactory.of(97.8903, 125.0242, 0);
    private final Pose collectFlower           = poseFactory.of(129, 90, 0);
    private final Pose shoot3Part1             = poseFactory.of(119, 90, 0);
    private final Pose shoot3Part2Ctrl         = poseFactory.of(127.2754, 21.3166, 0);
    private final Pose shoot3                  = poseFactory.of(85, 18.5, 90);
    private final Pose park1                   = poseFactory.of(132.2288, 44.9007, 180);

    // position 2 (Tag ID = 2)
    private final Pose start2                      = poseFactory.of(60, 133, 270);
    private final Pose shoot1_2                    = poseFactory.of(58, 123, 270);
    private final Pose collectHomeFlowerPart1      = poseFactory.of(48, 120, 90);
    private final Pose collectHomeFlowerPart2      = poseFactory.of(48, 124.5, 90);
    private final Pose shoot2Part1                 = poseFactory.of(48, 118, 90);
    private final Pose shoot2Part2Start            = poseFactory.of(48, 118, 270);
    private final Pose shoot2_2                    = poseFactory.of(58, 123, 270);
    private final Pose park2                       = poseFactory.of(11, 118, 0);

    private int selectedTag = 1;

    private void dontShoot() {
        leftCR.setPower(0);
        rightCR.setPower(0);
        integration.setPower(0);
    }
    public void lowerDolphin() {
        leftDolphin.setPosition(0.535);
        rightDolphin.setPosition(0.55);
    }
    public void liftDolphin() {
        leftDolphin.setPosition(0.1);
        rightDolphin.setPosition(0.1);
    }
    public void shoot() {
        leftCR.setPower(-1);
        rightCR.setPower(1);
        integration.setPower(1);
    }
    public Command autoRoutine1() {
        return sequential(
                instant(this::liftDolphin),
                waitMs(1000),
                follow(follower, pathShoot1()),
                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
                instant(this::dontShoot),

                follow(follower, pathCollectGardenPart1()),
                follow(follower, pathCollectGardenPart2()),
                waitMs(1000),
                follow(follower, pathShoot2()),
                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
                instant(this::dontShoot),
                instant(this::lowerDolphin),

                follow(follower, pathCollectFlower()),
                waitMs(COLLECT_FLOWER_TIME_MS),
                follow(follower, pathShoot3Part1()),
                instant(this::liftDolphin),
                follow(follower, pathShoot3()),
                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
                instant(this::dontShoot),

                follow(follower, pathPark1())
        );
    }
    public Command autoRoutine2() {
        return sequential(
                instant(this::liftDolphin),
                waitMs(500),
                follow(follower, pathShoot1_2()),
                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
                instant(this::dontShoot),

                // Turn + collect home flower
//                follow(follower, pathCollectHomeFlowerPart1Start()),
                follow(follower, pathCollectHomeFlowerPart1()),
                follow(follower, pathCollectHomeFlowerPart2()),
                waitMs(800),

                // Go back to shoot
                follow(follower, pathShoot2Part1()),
                follow(follower, pathShoot2Part2Start()),
                follow(follower, pathShoot2_2()),
                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
                instant(this::dontShoot),

                // Park
                follow(follower, pathPark2())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);

        // Hardware
        leftCR       = hardwareMap.get(CRServo.class, "leftCR");
        rightCR      = hardwareMap.get(CRServo.class, "rightCR");
        leftDolphin  = hardwareMap.get(Servo.class, "leftDolphin");
        rightDolphin = hardwareMap.get(Servo.class, "rightDolphin");
        leftDolphin.setDirection(Servo.Direction.REVERSE);

        intake      = hardwareMap.get(DcMotor.class, "Intake");
        shooter     = hardwareMap.get(DcMotorEx.class, "Shooter");
        integration = hardwareMap.get(DcMotor.class, "Integration");

        shooter.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);   // AprilTag pipeline
        limelight.start();

        while (!isStarted() && !isStopRequested()) {
            llResult = limelight.getLatestResult();

            if (llResult != null && llResult.isValid() && !llResult.getFiducialResults().isEmpty()) {
                selectedTag = llResult.getFiducialResults().get(0).getFiducialId();
                telemetry.addData("AprilTag ID", selectedTag);
            } else {
                telemetry.addLine("No AprilTag detected – defaulting to Tag 1");
            }
            telemetry.addData("Selected Routine", selectedTag == 2 ? "Task 2" : "Action 1");
            telemetry.update();
        }

        if (selectedTag == 1) {
            follower.setPose(start1);
        } else {
            follower.setPose(start2);
        }
        follower.update();

        if (selectedTag == 2) {
            schedule(autoRoutine2());
        } else {
            schedule(autoRoutine1());
        }

        while (opModeIsActive()) {
            intake.setPower(-1);
            shooter.setVelocity(SHOOTER_VELOCITY);

            follower.update();
            Scheduler.execute();

            telemetry.addData("Routine", selectedTag == 2 ? "Task 2" : "Task 1");
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            if (follower.currentPath() != null) {
                telemetry.addData("Distance remaining", follower.distanceToEndpoint());
            }
            telemetry.update();
        }
    }
    //path 1
    public Path pathShoot1() {
        return line(start1, shoot1).linear(start1, shoot1);
    }
    public Path pathCollectGardenPart1() {
        return curve(shoot1, collectGardenPart1Ctrl, collectGardenPart1)
                .linear(shoot1, collectGardenPart1);
    }
    public Path pathCollectGardenPart2() {
        return line(collectGardenPart1, collectGardenPart2)
                .linear(collectGardenPart1, collectGardenPart2);
    }
    public Path pathShoot2() {
        return curve(collectGardenPart2, shoot2Ctrl, shoot2)
                .linear(collectGardenPart2, shoot2);
    }
    public Path pathCollectFlower() {
        return line(shoot2, collectFlower).linear(shoot2, collectFlower);
    }
    public Path pathShoot3Part1() {
        return line(collectFlower, shoot3Part1).reverseTangent();
    }
    public Path pathShoot3() {
        return curve(shoot3Part1, shoot3Part2Ctrl, shoot3)
                .linear(shoot3Part1, shoot3);
    }
    public Path pathPark1() {
        return line(shoot3, park1).linear(shoot3, park1);
    }
   //path 2
    public Path pathShoot1_2() {
        return line(start2, shoot1_2).linear(start2, shoot1_2);
    }
//    public Path pathCollectHomeFlowerPart1Start() {
//        return line(shoot1_2, collectHomeFlowerPart1Start).linear ( shoot1_2, collectHomeFlowerPart1Start );
//    }
    public Path pathCollectHomeFlowerPart1() {
        return line(shoot1_2, collectHomeFlowerPart1)
                .linear(shoot1_2, collectHomeFlowerPart1);
    }
    public Path pathCollectHomeFlowerPart2() {
        return line(collectHomeFlowerPart1, collectHomeFlowerPart2).linear(collectHomeFlowerPart1, collectHomeFlowerPart2);
    }
    public Path pathShoot2Part1() {
        return line(collectHomeFlowerPart2, shoot2Part1).linear(collectHomeFlowerPart2, shoot2Part1);
    }
    public Path pathShoot2Part2Start() {
        return line(shoot2Part1, shoot2Part2Start).linear(shoot2Part1, shoot2Part2Start);
    }
    public Path pathShoot2_2() {
        return line(shoot2Part2Start, shoot2_2).linear(shoot2Part2Start, shoot2_2);
    }
    public Path pathPark2() {
        return line(shoot2_2, park2).linear(shoot2_2, park2);
    }
}