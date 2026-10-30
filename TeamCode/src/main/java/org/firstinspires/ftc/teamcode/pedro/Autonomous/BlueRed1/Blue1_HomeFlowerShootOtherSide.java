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

    private DcMotor intake, integration;
    private DcMotorEx shooter;
    private CRServo leftCR, rightCR;

    private Servo leftDolphin, rightDolphin;

    boolean is_lowered = false;

    private static final double SHOOTER_VELOCITY = 1350;
    private final double Kp = 255.0, Ki = 0.0, Kd = 0.0, Kf = 11.62;
    private static final long SHOOT_TIME_MS = 3000;
    private static final long COLLECT_FLOWER_TIME_MS = 2000;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(82.5, 133, 270);
    private final Pose shoot1 = poseFactory.of(84, 123, 270);
    private final Pose collectGardenPart1Start = poseFactory.of(84, 123, 270);
    private final Pose collectGardenPart1 = poseFactory.of(120, 132, 0);
    private final Pose collectGardenPart2Start = poseFactory.of(120, 132, 0);
    private final Pose collectGardenPart2 = poseFactory.of(132, 132, 0);
    private final Pose shoot2Start = poseFactory.of(132, 132, 0);
    private final Pose shoot2 = poseFactory.of(84, 123, 270);
    private final Pose collectHomeFlowerStart = poseFactory.of(84, 123, 270);
    private final Pose collectHomeFlower = poseFactory.of(129, 94.5, 0);
    private final Pose shoot3Part1Start = poseFactory.of(129, 94.5, 0);
    private final Pose shoot3Part1 = poseFactory.of(118, 65, 90);
    private final Pose shoot3Part1Control1 = poseFactory.of(118, 90, 0);
    private final Pose shoot3Part2Start = poseFactory.of(118, 65, 90);
    private final Pose shoot3Part2 = poseFactory.of(84, 18, 90);
    private final Pose shoot3Part2Control1 = poseFactory.of(118, 30, 0);
    private final Pose parkStart = poseFactory.of(84, 18, 90);
    private final Pose park = poseFactory.of(131, 46, 180);
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
                follow(follower, shoot1()),
                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
                instant(this::dontShoot),
                follow(follower, collectGardenPart1()),
                follow(follower, collectGardenPart2()),
                follow(follower, shoot2()),
                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
                instant(this::dontShoot),
                instant(this::lowerDolphin),
                follow(follower, collectHomeFlower()),
                waitMs(COLLECT_FLOWER_TIME_MS),
                follow(follower, shoot3Part1()),
                instant(this::liftDolphin),
                follow(follower, shoot3Part2()),
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
        return line(start, shoot1).linear(start, shoot1);
    }

    public Path collectGardenPart1() {
        return line(collectGardenPart1Start, collectGardenPart1).linear(collectGardenPart1Start, collectGardenPart1);
    }

    public Path collectGardenPart2() {
        return line(collectGardenPart2Start, collectGardenPart2).linear(collectGardenPart2Start, collectGardenPart2);
    }

    public Path shoot2() {
        return line(shoot2Start, shoot2).linear(shoot2Start, shoot2);
    }

    public Path collectHomeFlower() {
        return line(collectHomeFlowerStart, collectHomeFlower).linear(collectHomeFlowerStart, collectHomeFlower);
    }

    public Path shoot3Part1() {
        return curve(shoot3Part1Start, shoot3Part1Control1, shoot3Part1).linear(shoot3Part1Start, shoot3Part1);
    }

    public Path shoot3Part2() {
        return curve(shoot3Part2Start, shoot3Part2Control1, shoot3Part2).linear(shoot3Part2Start, shoot3Part2);
    }

    public Path park() {
        return curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
}
