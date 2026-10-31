package org.firstinspires.ftc.teamcode.pedro.Autonomous.BlueRed2;

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

@Autonomous(name = "Blue - Red 2", group = "Blue2")
public class BlueRed2 extends LinearOpMode {

    private Follower follower;

    private boolean is_lowered = false;

    private DcMotor intake, integration;
    private DcMotorEx shooter;
    private CRServo leftCR, rightCR;

    private Servo leftDolphin, rightDolphin;

    private static final double SHOOTER_VELOCITY = 1350;
    private final double Kp = 255.0, Ki = 0.0, Kd = 0.0, Kf = 11.62;
    private static final long SHOOT_TIME_MS = 3000;
    private static final long COLLECT_FLOWER_TIME_MS = 3000;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(60, 133, 270);
    private final Pose shoot1 = poseFactory.of(58, 123, 270);
    private final Pose collectHomeFlowerPart1Start = poseFactory.of(58, 123, 90);
    private final Pose collectHomeFlowerPart1 = poseFactory.of(48, 120, 90);
    private final Pose collectHomeFlowerPart2 = poseFactory.of(48, 124.5, 90);
    private final Pose shoot2Part1 = poseFactory.of(48, 118, 90);
    private final Pose shoot2Part2Start = poseFactory.of(48, 118, 270);
    private final Pose shoot2 = poseFactory.of(58, 123, 270);
    private final Pose park = poseFactory.of(11, 118, 0);

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
    // Autonomous Routine
    public Command autoRoutine() {
        return sequential(
                waitMs(10000),
                follow(follower, shoot1()),
                //wait for hive to tip (Limelight AprilTag)
                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
                instant(this::dontShoot),
                follow(follower, collectHomeFlowerPart1()),
                instant(this::lowerDolphin),
                follow(follower, collectHomeFlowerPart2()),
                waitMs(COLLECT_FLOWER_TIME_MS),
                follow(follower, shoot2Part1()),
                instant(this::liftDolphin),
                waitMs(2000),
                follow(follower, shoot2Part2()),
                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
                instant(this::dontShoot),
                follow(follower, park())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
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
        schedule(autoRoutine());

        while (opModeIsActive()) {

            intake.setPower(-1);
            shooter.setVelocity(SHOOTER_VELOCITY);

            follower.update();
            Scheduler.execute();

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

    public Path shoot1() {
        return Paths.line(start, shoot1).linear(start, shoot1);
    }

    public Path collectHomeFlowerPart1() {
        return Paths.line(collectHomeFlowerPart1Start, collectHomeFlowerPart1).linear(collectHomeFlowerPart1Start, collectHomeFlowerPart1);
    }

    public Path collectHomeFlowerPart2() {
        return Paths.line(collectHomeFlowerPart1, collectHomeFlowerPart2).linear(collectHomeFlowerPart1, collectHomeFlowerPart2);
    }

    public Path shoot2Part1() {
        return Paths.line(collectHomeFlowerPart2, shoot2Part1).linear(collectHomeFlowerPart2, shoot2Part1);
    }

    public Path shoot2Part2() {
        return Paths.line(shoot2Part2Start, shoot2).linear(shoot2Part2Start, shoot2);
    }

    public Path park() {
        return Paths.line(shoot2, park).linear(shoot2, park);
    }
}
