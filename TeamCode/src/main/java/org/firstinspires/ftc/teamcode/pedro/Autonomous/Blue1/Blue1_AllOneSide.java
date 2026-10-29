package org.firstinspires.ftc.teamcode.pedro.Autonomous.Blue1;

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

@Autonomous(name = "Blue1_AllOneSide", group = "Blue1")
public class Blue1_AllOneSide extends LinearOpMode {

    private Follower follower;

    private DcMotor intake, integration;
    private DcMotorEx shooter;
    private CRServo leftCR, rightCR;

    private Servo leftDolphin, rightDolphin;

    private static final double SHOOTER_VELOCITY = 1350;
    private final double Kp = 255.0, Ki = 0.0, Kd = 0.0, Kf = 11.62;
    private static final long SHOOT_TIME_MS = 3000;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(85, 132, 270);
    private final Pose shoot1 = poseFactory.of(85, 123, 270);
    private final Pose collectGardenPart1 = poseFactory.of(119.3584, 133, 0);
    private final Pose collectGardenPart1Control1 = poseFactory.of(98.2245, 124.082, 0);
    private final Pose collectGardenPart2 = poseFactory.of(132.6304, 133.0665, 0);
    private final Pose shoot2 = poseFactory.of(85, 123, 270);
    private final Pose shoot2Control1 = poseFactory.of(97.8903, 125.0242, 0);
    private final Pose collectFlower = poseFactory.of(129, 94.5, 0);
    private final Pose shoot3 = poseFactory.of(85, 123, 270);
    private final Pose park = poseFactory.of(132.2288, 44.9007, 180);
    private final Pose parkControl1 = poseFactory.of(132.4007, 93.0769, 0);

    // Autonomous routine
    private void startFeed() {
        leftCR.setPower(-1);
        rightCR.setPower(1);
        integration.setPower(1);
    }

    private void stopFeed() {
        leftCR.setPower(0);
        rightCR.setPower(0);
        integration.setPower(0);
    }

    private void lowerDolphin(){
        leftDolphin.setPosition(0.535);
        rightDolphin.setPosition(0.8);
    }
    private void liftDolphin(){
        leftDolphin.setPosition(0.15);
        rightDolphin.setPosition(0.25);
    }

    // Feed balls for SHOOT_TIME_MS, then stop
    private Command shootSequence() {
        return sequential(
                instant(this::startFeed),
                waitMs(SHOOT_TIME_MS),
                instant(this::stopFeed)
        );
    }
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot1()),
                // Shoot at shoot 1 position
                instant(this::startFeed),
                waitMs(SHOOT_TIME_MS),
                instant(this::stopFeed),
                // End  Shoot routine and go to Garden Part1
                follow(follower, collectGardenPart1()),
                follow(follower, collectGardenPart2()),
                follow(follower, shoot2()),
                // Shoot at shoot 1 position
                instant(this::startFeed),
                waitMs(SHOOT_TIME_MS),
                instant(this::stopFeed),
                instant(this::lowerDolphin),
                // End  Shoot routine and go to Flower
                follow(follower, collectFlower()),
                waitMs(2000),
                follow(follower, shoot3()),
                instant(this::liftDolphin),
                // Shoot at shoot 1 position
                instant(this::startFeed),
                waitMs(SHOOT_TIME_MS),
                instant(this::stopFeed),
                // End  Shoot routine and go to Park
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
        return curve(shoot1, collectGardenPart1Control1, collectGardenPart1).linear(shoot1, collectGardenPart1);
    }

    public Path collectGardenPart2() {
        return line(collectGardenPart1, collectGardenPart2).linear(collectGardenPart1, collectGardenPart2);
    }

    public Path shoot2() {
        return curve(collectGardenPart2, shoot2Control1, shoot2).linear(collectGardenPart2, shoot2);
    }

    public Path collectFlower() {
        return line(shoot2, collectFlower).linear(shoot2, collectFlower);
    }

    public Path shoot3() {
        return line(collectFlower, shoot3).linear(collectFlower, shoot3);
    }

    public Path park() {
        return curve(shoot3, parkControl1, park).linear(shoot3, park);
    }
}
