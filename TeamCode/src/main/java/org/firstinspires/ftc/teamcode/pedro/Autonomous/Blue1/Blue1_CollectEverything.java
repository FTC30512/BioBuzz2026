package org.firstinspires.ftc.teamcode.pedro.Autonomous.Blue1;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Blue1_CollectEverything", group = "Blue1")
public class Blue1_CollectEverything extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(82.5, 132, 270);
    private final Pose shoot1 = poseFactory.of(84, 123, 270);
    private final Pose collectGardenPart1Start = poseFactory.of(84, 123, 270);
    private final Pose collectGardenPart1 = poseFactory.of(120, 132, 0);
    private final Pose collectGardenPart1Control1 = poseFactory.of(98.0061, 129.4182, 0);
    private final Pose collectGardenPart2Start = poseFactory.of(120, 132, 0);
    private final Pose collectGardenPart2 = poseFactory.of(132, 132, 0);
    private final Pose shoot2Part1Start = poseFactory.of(132, 132, 0);
    private final Pose shoot2Part1 = poseFactory.of(112.3637, 73.9032, 90);
    private final Pose shoot2Part1Control1 = poseFactory.of(112.8926, 118.7277, 0);
    private final Pose shoot2Part2Start = poseFactory.of(112.3637, 73.9032, 90);
    private final Pose shoot2Part2 = poseFactory.of(84, 21, 90);
    private final Pose shoot2Part2Control1 = poseFactory.of(108.6728, 22.4148, 0);
    private final Pose collectAwayFlowerStart = poseFactory.of(84, 21, 270);
    private final Pose collectAwayFlower = poseFactory.of(94, 14, 270);
    private final Pose collectAwayFlowerControl1 = poseFactory.of(94, 20.8888, 0);
    private final Pose shoot3Part1Start = poseFactory.of(94, 14, 270);
    private final Pose shoot3Part1 = poseFactory.of(84.318, 18, 270);
    private final Pose shoot3Part1Control1 = poseFactory.of(93.6449, 17.9303, 0);
    private final Pose shoot3Part2Start = poseFactory.of(84.318, 18, 90);
    private final Pose shoot3Part2 = poseFactory.of(84.3685, 21, 90);
    private final Pose collectBaseFlowerPart1Start = poseFactory.of(84.3685, 21, 90);
    private final Pose collectBaseFlowerPart1 = poseFactory.of(114.4067, 59.9494, 0);
    private final Pose collectBaseFlowerPart1Control1 = poseFactory.of(114.3326, 25.2708, 0);
    private final Pose collectBaseFlowerPart2Start = poseFactory.of(114.4067, 59.9494, 0);
    private final Pose collectBaseFlowerPart2 = poseFactory.of(124, 94, 0);
    private final Pose collectBaseFlowerPart2Control1 = poseFactory.of(117.4708, 93.1202, 0);
    private final Pose shoot4Start = poseFactory.of(124, 94, 0);
    private final Pose shoot4 = poseFactory.of(84, 123, 270);
    private final Pose shoot4Control1 = poseFactory.of(97.7742, 101.636, 0);
    private final Pose parkStart = poseFactory.of(84, 123, 270);
    private final Pose park = poseFactory.of(130, 46, 180);
    private final Pose parkControl1 = poseFactory.of(123.7382, 112.8022, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot1()),
                follow(follower, collectGardenPart1()),
                follow(follower, collectGardenPart2()),
                follow(follower, shoot2Part1()),
                follow(follower, shoot2Part2()),
                follow(follower, collectAwayFlower()),
                follow(follower, shoot3Part1()),
                follow(follower, shoot3Part2()),
                follow(follower, collectBaseFlowerPart1()),
                follow(follower, collectBaseFlowerPart2()),
                follow(follower, shoot4()),
                follow(follower, park())
        );
    }

    @Override
    public void runOpMode() {
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            shooter.setPower(-1);
            intake.setPower(1);

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
        return Paths.curve(collectGardenPart1Start, collectGardenPart1Control1, collectGardenPart1).linear(collectGardenPart1Start, collectGardenPart1);
    }

    public Path collectGardenPart2() {
        return Paths.line(collectGardenPart2Start, collectGardenPart2).linear(collectGardenPart2Start, collectGardenPart2);
    }

    public Path shoot2Part1() {
        return Paths.curve(shoot2Part1Start, shoot2Part1Control1, shoot2Part1).linear(shoot2Part1Start, shoot2Part1);
    }

    public Path shoot2Part2() {
        return Paths.curve(shoot2Part2Start, shoot2Part2Control1, shoot2Part2).linear(shoot2Part2Start, shoot2Part2);
    }

    public Path collectAwayFlower() {
        return Paths.curve(collectAwayFlowerStart, collectAwayFlowerControl1, collectAwayFlower).linear(collectAwayFlowerStart, collectAwayFlower);
    }

    public Path shoot3Part1() {
        return Paths.curve(shoot3Part1Start, shoot3Part1Control1, shoot3Part1).linear(shoot3Part1Start, shoot3Part1);
    }

    public Path shoot3Part2() {
        return Paths.line(shoot3Part2Start, shoot3Part2).linear(shoot3Part2Start, shoot3Part2);
    }

    public Path collectBaseFlowerPart1() {
        return Paths.curve(collectBaseFlowerPart1Start, collectBaseFlowerPart1Control1, collectBaseFlowerPart1).linear(collectBaseFlowerPart1Start, collectBaseFlowerPart1);
    }

    public Path collectBaseFlowerPart2() {
        return Paths.curve(collectBaseFlowerPart2Start, collectBaseFlowerPart2Control1, collectBaseFlowerPart2).linear(collectBaseFlowerPart2Start, collectBaseFlowerPart2);
    }

    public Path shoot4() {
        return Paths.curve(shoot4Start, shoot4Control1, shoot4).linear(shoot4Start, shoot4);
    }

    public Path park() {
        return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
}
