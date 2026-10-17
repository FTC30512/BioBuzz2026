//CHANGE COLLECFLOWER PATHS TO NOT BUMP INTO FLOWERS

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

@Autonomous(name = "Blue1_BothSidesTwoFlowers", group = "Blue1")
public class Blue1_BothSidesTwoFlowers extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(82, 132, 270);
    private final Pose shoot1 = poseFactory.of(84, 123, 270);
    private final Pose collectPollenStart = poseFactory.of(84, 123, 270);
    private final Pose collectPollen = poseFactory.of(124, 94, 0);
    private final Pose collectPollenControl1 = poseFactory.of(101.8922, 99.3089, 0);
    private final Pose shoot2Part1Start = poseFactory.of(124, 94, 0);
    private final Pose shoot2Part1 = poseFactory.of(118.1652, 64.9325, 90);
    private final Pose shoot2Part1Control1 = poseFactory.of(118.2555, 89.598, 0);
    private final Pose shoot2Part2Start = poseFactory.of(118.1652, 64.9325, 90);
    private final Pose shoot2Part2 = poseFactory.of(83.3621, 28.5029, 90);
    private final Pose shoot2Part2Control1 = poseFactory.of(118.3103, 26.944, 0);
    private final Pose collectFlower2Part2Start = poseFactory.of(83.3621, 28.5029, 270);
    private final Pose collectFlower2Part2 = poseFactory.of(94, 14, 270);
    private final Pose collectFlower2Part2Control1 = poseFactory.of(94.2523, 26.0079, 0);
    private final Pose parkPart1Start = poseFactory.of(94, 14, 270);
    private final Pose parkPart1 = poseFactory.of(94, 20, 270);
    private final Pose parkPart2Start = poseFactory.of(94, 20, 115);
    private final Pose parkPart2 = poseFactory.of(130, 46, 180);
    private final Pose parkPart2Control1 = poseFactory.of(112.4971, 42.1739, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot1()),
                follow(follower, collectPollen()),
                follow(follower, shoot2Part1()),
                follow(follower, shoot2Part2()),
                follow(follower, collecTflower2Part2()),
                follow(follower, parkPart1()),
                follow(follower, parkPart2())
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

    public Path collectPollen() {
        return Paths.curve(collectPollenStart, collectPollenControl1, collectPollen).linear(collectPollenStart, collectPollen);
    }

    public Path shoot2Part1() {
        return Paths.curve(shoot2Part1Start, shoot2Part1Control1, shoot2Part1).linear(shoot2Part1Start, shoot2Part1);
    }

    public Path shoot2Part2() {
        return Paths.curve(shoot2Part2Start, shoot2Part2Control1, shoot2Part2).linear(shoot2Part2Start, shoot2Part2);
    }

    public Path collecTflower2Part2() {
        return Paths.curve(collectFlower2Part2Start, collectFlower2Part2Control1, collectFlower2Part2).linear(collectFlower2Part2Start, collectFlower2Part2);
    }

    public Path parkPart1() {
        return Paths.line(parkPart1Start, parkPart1).linear(parkPart1Start, parkPart1);
    }

    public Path parkPart2() {
        return Paths.curve(parkPart2Start, parkPart2Control1, parkPart2).linear(parkPart2Start, parkPart2);
    }
}
