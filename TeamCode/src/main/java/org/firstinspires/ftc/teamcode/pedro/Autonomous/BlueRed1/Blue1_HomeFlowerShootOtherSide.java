package org.firstinspires.ftc.teamcode.pedro.Autonomous.BlueRed1;

import static com.pedropathing.api.Paths.*;

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

@Autonomous(name = "Blue1_HomeFlowerShootOtherSide", group = "Blue1")
public class Blue1_HomeFlowerShootOtherSide extends LinearOpMode {

    private Follower follower;

    boolean is_lowered = false;
	
    private DcMotor intake, integration;
    private DcMotorEx shooter;
    private CRServo leftCR, rightCR;

    private Servo leftDolphin, rightDolphin;

    private static final double SHOOTER_VELOCITY = 1350;
    private final double Kp = 255.0, Ki = 0.0, Kd = 0.0, Kf = 11.62;
    private static final long SHOOT_TIME_MS = 3000;
    private static final long COLLECT_FLOWER_TIME_MS = 2000;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(85, 132, 270);
    private final Pose shoot1 = poseFactory.of(85, 123, 270);
    private final Pose collectGardenPart1Start = poseFactory.of(84, 123, 270);
    private final Pose collectGardenPart1 = poseFactory.of(119.35, 132, 0);
    private final Pose collectGardenPart2Start = poseFactory.of(120, 132, 0);
    private final Pose collectGardenPart2 = poseFactory.of(132, 132, 0);
    private final Pose shoot2Start = poseFactory.of(132, 132, 0);
    private final Pose shoot2 = poseFactory.of(85, 123, 270);
    private final Pose collectFlowerPart1Start = poseFactory.of(84, 123, 270);
    private final Pose collectFlowerPart1 = poseFactory.of(119, 94.5, 0);
    private final Pose collectFlowerPart2Start = poseFactory.of(119, 94.5, 0);
    private final Pose collectFlowerPart2 = poseFactory.of(127, 94.3417, 0);
    private final Pose shoot3Part1 = poseFactory.of(119, 94.5, -0.9832);
    private final Pose shoot3Part2Start = poseFactory.of(119, 94.5, 0);
    private final Pose shoot3Part2 = poseFactory.of(118, 65, 90);
    private final Pose shoot3Part3Start = poseFactory.of(118, 65, 90);
    private final Pose shoot3Part3 = poseFactory.of(84, 18, 90);
    private final Pose shoot3Part3Control1 = poseFactory.of(118, 30, 0);
    private final Pose parkStart = poseFactory.of(84, 18, 90);
    private final Pose park = poseFactory.of(131.1974, 45.4079, 180);
    private final Pose parkControl1 = poseFactory.of(118, 30, 0);

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
                waitMs(1000),
                follow(follower, shoot1()),
                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
                instant(this::dontShoot),
                follow(follower, collectGardenPart1()),
                waitMs(250),
                follow(follower, collectGardenPart2()),
                follow(follower, shoot2()),
                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
                instant(this::dontShoot),
                follow(follower, collectFlowerPart1()),
                instant(this::lowerDolphin),
                follow(follower, collectFlowerPart2()),
                waitMs(COLLECT_FLOWER_TIME_MS),
                follow(follower, shoot3Part1()),
                instant(this::liftDolphin),
                follow(follower, shoot3Part2()),
                follow(follower, shoot3Part3()),
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

    public Path collectGardenPart1() {
        return Paths.line(collectGardenPart1Start, collectGardenPart1).linear(collectGardenPart1Start, collectGardenPart1);
    }

    public Path collectGardenPart2() {
        return Paths.line(collectGardenPart2Start, collectGardenPart2).linear(collectGardenPart2Start, collectGardenPart2);
    }

    public Path shoot2() {
        return Paths.line(shoot2Start, shoot2).linear(shoot2Start, shoot2);
    }

    public Path collectFlowerPart1() {
        return Paths.line(collectFlowerPart1Start, collectFlowerPart1).linear(collectFlowerPart1Start, collectFlowerPart1);
    }

    public Path collectFlowerPart2() {
        return Paths.line(collectFlowerPart2Start, collectFlowerPart2).linear(collectFlowerPart2Start, collectFlowerPart2);
    }

    public Path shoot3Part1() {
        return Paths.line(collectFlowerPart2, shoot3Part1).reverseTangent();
    }

    public Path shoot3Part2() {
        return Paths.line(shoot3Part2Start, shoot3Part2).linear(shoot3Part2Start, shoot3Part2);
    }

    public Path shoot3Part3() {
        return Paths.curve(shoot3Part3Start, shoot3Part3Control1, shoot3Part3).linear(shoot3Part3Start, shoot3Part3);
    }

    public Path park() {
        return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
}
