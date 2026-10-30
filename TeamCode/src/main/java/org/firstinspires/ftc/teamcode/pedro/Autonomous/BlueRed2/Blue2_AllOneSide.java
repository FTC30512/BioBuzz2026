package org.firstinspires.ftc.teamcode.pedro.Autonomous.BlueRed2;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Blue2_AllOneSide", group = "Blue2")
public class Blue2_AllOneSide extends LinearOpMode {

    private Follower follower;

    private DcMotor intake, integration;
    private DcMotorEx shooter;
    private CRServo leftCR, rightCR;

    private Servo leftDolphin, rightDolphin;

    boolean is_lowered = false;

    private static final double SHOOTER_VELOCITY = 1350;
    private final double Kp = 255.0, Ki = 0.0, Kd = 0.0, Kf = 11.62;
    private static final long SHOOT_TIME_MS = 3000;
    private static final long COLLECT_FLOWER_TIME_MS = 2000;

    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorY(70.75);

    private final Pose start = poseFactory.of(81.5, 133, -90);
    private final Pose shoot1 = poseFactory.of(83.5, 117, -90);
    private final Pose collectHomeFlowerStart = poseFactory.of(83.5, 117, 90);
    private final Pose collectHomeFlower = poseFactory.of(94.5, 126, 90);
    private final Pose collectHomeFlowerControl1 = poseFactory.of(94.5, 117, 0);
    private final Pose shoot2Part1Start = poseFactory.of(94.5, 126, 90);
    private final Pose shoot2Part1 = poseFactory.of(94.5, 120, 90);
    private final Pose shoot2Part2Start = poseFactory.of(94.5, 120, -90);
    private final Pose shoot2Part2 = poseFactory.of(83.5, 117, -90);
    private final Pose shoot2Part2Control1 = poseFactory.of(94.5, 117, 0);
    private final Pose parkStart = poseFactory.of(83.5, 117, -90);
    private final Pose park = poseFactory.of(130.5, 117, 180);

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
                follow(follower, shoot1()),
                // Shoot at shoot1 position
                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
                instant(this::dontShoot),
                // Collect flower and shoot at shoot2 position
                instant(this::lowerDolphin),
                follow(follower, collectHomeFlower()),
                waitMs(COLLECT_FLOWER_TIME_MS),
                follow(follower, shoot2Part1()),
                instant(this::liftDolphin),
                follow(follower, shoot2Part2()),
                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
                instant(this::dontShoot),
                // Park
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
            shooter.setPower(SHOOTER_VELOCITY);
            intake.setPower(-1);

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

    public Path collectHomeFlower() {
        return Paths.curve(collectHomeFlowerStart, collectHomeFlowerControl1, collectHomeFlower).linear(collectHomeFlowerStart, collectHomeFlower);
    }

    public Path shoot2Part1() {
        return Paths.line(shoot2Part1Start, shoot2Part1).linear(shoot2Part1Start, shoot2Part1);
    }

    public Path shoot2Part2() {
        return Paths.curve(shoot2Part2Start, shoot2Part2Control1, shoot2Part2).linear(shoot2Part2Start, shoot2Part2);
    }

    public Path park() {
        return Paths.line(parkStart, park).linear(parkStart, park);
    }
}