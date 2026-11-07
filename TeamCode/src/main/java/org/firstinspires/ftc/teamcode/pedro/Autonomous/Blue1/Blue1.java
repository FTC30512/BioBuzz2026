package org.firstinspires.ftc.teamcode.pedro.Autonomous.Blue1;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.TeleOp.Movement;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Blue 1", group = "Blue1")
public class Blue1 extends LinearOpMode {

    private Follower follower;

    private boolean is_lowered = false;

    //Limelight
    private IMU imu;
    private GoBildaPinpointDriver pinpoint;
    Limelight3A limelight;
    private LLResult llResult;
    private boolean isTurning = false;
    // Motors & Servos
    private DcMotor intake, integration;
    private DcMotorEx shooter;
    private CRServo leftCR, rightCR;
    private Servo leftDolphin, rightDolphin;
    // Important Constants
    private static final double SHOOTER_VELOCITY = 1350;
    private final double Kp = 255.0, Ki = 0.0, Kd = 0.0, Kf = 11.62;
    private static final long SHOOT_TIME_MS = 2000;
    private static final long COLLECT_FLOWER_TIME_MS = 3000;
    private static final long LOWER_DOLPHIN_TIME_MS = 1000;
    private boolean done_execute = false;
    //Poses
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(82.7448, 132.3672, 270);
    private final Pose shoot1 = poseFactory.of(84, 123, 270);
    private final Pose collectGardenPart1Start = poseFactory.of(84, 123, 270);
    private final Pose collectGardenPart1 = poseFactory.of(120, 131.5, 0);
    private final Pose collectGardenPart2Start = poseFactory.of(120, 131.5, 0);
    private final Pose collectGardenPart2 = poseFactory.of(131, 131.5, 0);
    private final Pose shoot2Part1Start = poseFactory.of(131, 131.5, 0);
    private final Pose shoot2Part1 = poseFactory.of(109, 121.5, 270);
    private final Pose shoot2Part2Start = poseFactory.of(109, 121.5, 270);
    private final Pose shoot2Part2 = poseFactory.of(105, 121.5, 235);

    //Shoot Other Side
    private final Pose shoot3 = poseFactory.of(114, 18, 125);
    private final Pose collectAwayFlowerPart1Start = poseFactory.of(114, 18, 270);
    private final Pose collectAwayFlowerPart1 = poseFactory.of(94.5, 22.5, 270);
    private final Pose collectAwayFlowerPart2Start = poseFactory.of(94.5, 22.5, 270);
    private final Pose collectAwayFlowerPart2 = poseFactory.of(94.5, 13, 270);
    private final Pose shoot4Part1Start = poseFactory.of(94.5, 13, 270);
    private final Pose shoot4Part1 = poseFactory.of(95.4792, 22.5, 270);
    private final Pose shoot4Part2Start = poseFactory.of(95.4792, 22.5, 125);
    private final Pose shoot4Part2 = poseFactory.of(114, 17.5, 125);
    private final Pose parkStart = poseFactory.of(114, 17.5, 125);
    private final Pose park = poseFactory.of(131, 46, 180);
    private final Pose parkControl1 = poseFactory.of(114, 44, 0);
    //Shoot Same Side
    private final Pose collectHomeFlowerPart1B = poseFactory.of(120, 94.5, 0);
    private final Pose collectHomeFlowerPart2StartB = poseFactory.of(120, 94.5, 0);
    private final Pose collectHomeFlowerPart2B = poseFactory.of(128, 94.5, 0);
    private final Pose shoot4Part1StartB = poseFactory.of(128, 94.5, 0);
    private final Pose shoot4Part1B = poseFactory.of(120, 94, 0);
    private final Pose shoot4Part2StartB = poseFactory.of(120, 94, 0);
    private final Pose shoot4Part2B = poseFactory.of(114, 18, 125);
    private final Pose collectAwayFlowerPart1StartB = poseFactory.of(114, 18, 270);
    private final Pose collectAwayFlowerPart1B = poseFactory.of(94.5, 22.5, 270);
    private final Pose collectAwayFlowerPart2StartB = poseFactory.of(94.5, 22.5, 270);
    private final Pose collectAwayFlowerPart2B = poseFactory.of(94.5, 13, 270);
    private final Pose shoot5Part1StartB = poseFactory.of(94.5, 13, 270);
    private final Pose shoot5Part1B = poseFactory.of(95, 22.5, 270);
    private final Pose shoot5Part2StartB = poseFactory.of(95, 22.5, 125);
    private final Pose shoot5Part2B = poseFactory.of(114, 17.5, 125);
    private final Pose parkStartB = poseFactory.of(114, 17.5, 125);
    private final Pose parkB = poseFactory.of(131, 46, 180);
    private final Pose parkControl1B = poseFactory.of(114, 44, 0);

    // Commands
    private void dontShoot() {
        leftCR.setPower(0);
        rightCR.setPower(0);
        integration.setPower(0);
    }

    public void lowerDolphin(){
        leftDolphin.setPosition(0.535);
        rightDolphin.setPosition(0.55);
        is_lowered = true;
    }
    public void liftDolphin(){
        leftDolphin.setPosition(0.1);
        rightDolphin.setPosition(0.1);
        is_lowered = false;
    }
    // --- Shooting method ---
    public void shoot() {
        leftCR.setPower(-1);
        rightCR.setPower(1);
        integration.setPower(1);
    }
    public void doneExecute() {
        done_execute = true;}
    // Autonomous Routines
    public Command Start() {
        return sequential(
                instant(this::liftDolphin),
                follow(follower, shoot1()),
                waitMs(1000),
                //Shoot
                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
                instant(this::dontShoot),
                //Collect Garden
                follow(follower, collectGardenPart1()),
                follow(follower, collectGardenPart2()),
                //Get Ready to Shoot Again
                follow(follower, shoot2Part1()),
                follow(follower, shoot2Part2()),
                instant(this::doneExecute)
        );
    }
    public Command ShootOtherSide() {
        return sequential(
                //go to other side and shoot
                follow(follower, shoot3()),
                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
                instant(this::dontShoot),
                // collect away flower
                follow(follower, collectAwayFlowerPart1()),
                instant(this::lowerDolphin),
                waitMs(LOWER_DOLPHIN_TIME_MS),
                follow(follower, collectAwayFlowerPart2()),
                waitMs(COLLECT_FLOWER_TIME_MS),
                follow(follower, shoot4Part1()),
                instant(this::liftDolphin),
                //shoot on other side for second tip
                follow(follower, shoot4Part2()),
                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
                instant(this::dontShoot),
                follow(follower, park())
        );
    }
    public Command ShootSameSide() {
        return sequential(
                // Shoot on same side to tip hive
                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
                instant(this::dontShoot),
                // collect home flower
                follow(follower, collectHomeFlowerPart1B()),
                instant(this::lowerDolphin),
                waitMs(LOWER_DOLPHIN_TIME_MS),
                follow(follower, collectHomeFlowerPart2B()),
                waitMs(COLLECT_FLOWER_TIME_MS),
                follow(follower, shoot4Part1B()),
                instant(this::liftDolphin),
                // shoot on other side
                follow(follower, shoot4Part2B()),
                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
                instant(this::dontShoot),
                // collect away flower
                follow(follower, collectAwayFlowerPart1B()),
                instant(this::lowerDolphin),
                waitMs(LOWER_DOLPHIN_TIME_MS),
                follow(follower, collectAwayFlowerPart2B()),
                waitMs(COLLECT_FLOWER_TIME_MS),
                follow(follower, shoot5Part1B()),
                instant(this::liftDolphin),
                // shoot on other side to tip again
                follow(follower, shoot5Part2B()),
                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
                instant(this::dontShoot),
                // park
                follow(follower, parkB())
        );
    }
    @Override
    public void runOpMode() {
        Scheduler.reset();

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();

        telemetry.addLine("Ready");
        telemetry.update();

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        leftCR = hardwareMap.get(CRServo.class, "leftCR");
        rightCR = hardwareMap.get(CRServo.class, "rightCR");
        leftDolphin = hardwareMap.get(Servo.class, "leftDolphin");
        rightDolphin = hardwareMap.get(Servo.class, "rightDolphin");
        leftDolphin.setDirection(Servo.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "Intake");
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooter.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        integration = hardwareMap.get(DcMotor.class, "Integration");

        waitForStart();
        resetRuntime();
        schedule(Start());

        while (opModeIsActive()) {
            llResult = limelight.getLatestResult();
            telemetry.addLine("Runtime: " + getRuntime());
            intake.setPower(-1);
            shooter.setVelocity(SHOOTER_VELOCITY);

            follower.update();
            Scheduler.execute();
            if (done_execute) {
                done_execute = false;
                if (llResult != null && llResult.isValid()) {
                    if (!llResult.getFiducialResults().isEmpty()) {
                        telemetry.addLine("No Hive Tip");
                        Scheduler.schedule(ShootSameSide());
                    } else {
                        telemetry.addLine("Hive Tip Identified");
                        Scheduler.schedule(ShootOtherSide());
                    }
                } else {
                    telemetry.addLine("Hive Tip Identified");
                    Scheduler.schedule(ShootOtherSide());
                }
            }

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }
            telemetry.update();
        }
    }
    //Paths
    //Start
    public Path shoot1() {
        return Paths.line(start, shoot1).linear(start, shoot1);
    }

    public Path collectGardenPart1() {
        return Paths.line(collectGardenPart1Start, collectGardenPart1).linear(collectGardenPart1Start, collectGardenPart1);
    }

    public Path collectGardenPart2() {
        return Paths.line(collectGardenPart2Start, collectGardenPart2).linear(collectGardenPart2Start, collectGardenPart2);
    }

    public Path shoot2Part1() {
        return Paths.line(shoot2Part1Start, shoot2Part1).linear(shoot2Part1Start, shoot2Part1);
    }

    public Path shoot2Part2() {
        return Paths.line(shoot2Part2Start, shoot2Part2).linear(shoot2Part2Start, shoot2Part2);
    }
    //ShootOtherSide
    public Path shoot3() {
        return Paths.line(shoot2Part2, shoot3).linear(shoot2Part2, shoot3);
    }

    public Path collectAwayFlowerPart1() {
        return Paths.line(collectAwayFlowerPart1Start, collectAwayFlowerPart1).linear(collectAwayFlowerPart1Start, collectAwayFlowerPart1);
    }

    public Path collectAwayFlowerPart2() {
        return Paths.line(collectAwayFlowerPart2Start, collectAwayFlowerPart2).linear(collectAwayFlowerPart2Start, collectAwayFlowerPart2);
    }

    public Path shoot4Part1() {
        return Paths.line(shoot4Part1Start, shoot4Part1).linear(shoot4Part1Start, shoot4Part1);
    }

    public Path shoot4Part2() {
        return Paths.line(shoot4Part2Start, shoot4Part2).linear(shoot4Part2Start, shoot4Part2);
    }

    public Path park() {
        return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
    // ShootSameSide
    public Path collectHomeFlowerPart1B() {
        return Paths.line(shoot2Part2, collectHomeFlowerPart1B).linear(shoot2Part2, collectHomeFlowerPart1B);
    }

    public Path collectHomeFlowerPart2B() {
        return Paths.line(collectHomeFlowerPart2StartB, collectHomeFlowerPart2B).linear(collectHomeFlowerPart2StartB, collectHomeFlowerPart2B);
    }

    public Path shoot4Part1B() {
        return Paths.line(shoot4Part1StartB, shoot4Part1B).linear(shoot4Part1StartB, shoot4Part1B);
    }

    public Path shoot4Part2B() {
        return Paths.line(shoot4Part2StartB, shoot4Part2B).linear(shoot4Part2StartB, shoot4Part2B);
    }

    public Path collectAwayFlowerPart1B() {
        return Paths.line(collectAwayFlowerPart1StartB, collectAwayFlowerPart1B).linear(collectAwayFlowerPart1StartB, collectAwayFlowerPart1B);
    }

    public Path collectAwayFlowerPart2B() {
        return Paths.line(collectAwayFlowerPart2StartB, collectAwayFlowerPart2B).linear(collectAwayFlowerPart2StartB, collectAwayFlowerPart2B);
    }

    public Path shoot5Part1B() {
        return Paths.line(shoot5Part1StartB, shoot5Part1B).linear(shoot5Part1StartB, shoot5Part1B);
    }

    public Path shoot5Part2B() {
        return Paths.line(shoot5Part2StartB, shoot5Part2B).linear(shoot5Part2StartB, shoot5Part2B);
    }

    public Path parkB() {
        return Paths.curve(parkStartB, parkControl1B, parkB).linear(parkStartB, parkB);
    }
}
