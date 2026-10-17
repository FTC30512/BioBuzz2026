package org.firstinspires.ftc.teamcode.pedro.Autonomous.Blue1;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Blue1_OtherTeamCantShoot", group = "Blue1")
public class Blue1_OtherTeamCantShoot extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(84.3489, 132.8817, 270);
    private final Pose shoot1 = poseFactory.of(84.1045, 117.6252, 270);
    private final Pose collectGardenPart1Start = poseFactory.of(84.1045, 117.6252, 270);
    private final Pose collectGardenPart1 = poseFactory.of(121, 132, 0);
    private final Pose collectGardenPart1Control1 = poseFactory.of(93.4689, 127.4283, 0);
    private final Pose collectGardenPart2Start = poseFactory.of(121, 132, 0);
    private final Pose collectGardenPart2 = poseFactory.of(132, 132, 0);
    private final Pose shoot2Part1Start = poseFactory.of(132, 132, 0);
    private final Pose shoot2Part1 = poseFactory.of(82.9499, 94.1917, 90);
    private final Pose shoot2Part1Control1 = poseFactory.of(90.1494, 125.7677, 0);
    private final Pose shoot2Part2Start = poseFactory.of(82.9499, 94.1917, 90);
    private final Pose shoot2Part2 = poseFactory.of(84, 21, 90);
    private final Pose collectFlowerPart1Start = poseFactory.of(84, 21, 270);
    private final Pose collectFlowerPart1 = poseFactory.of(94, 13, 270);
    private final Pose collectFlowerPart1Control1 = poseFactory.of(93.7779, 21.0862, 0);
    private final Pose shoot3Part1Start = poseFactory.of(94, 13, 270);
    private final Pose shoot3Part1 = poseFactory.of(94, 21, 270);
    private final Pose shoot3Part2Start = poseFactory.of(94, 21, 270);
    private final Pose shoot3Part2 = poseFactory.of(95, 21, 105);
    private final Pose parkStart = poseFactory.of(95, 21, 105);
    private final Pose park = poseFactory.of(130, 46, 180);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot1()),
                follow(follower, collectGardenPart1()),
                follow(follower, collectGardenPart2()),
                follow(follower, shoot2Part1()),
                follow(follower, shoot2Part2()),
                follow(follower, collectFlowerPart1()),
                follow(follower, shoot3Part1()),
                follow(follower, shoot3Part2()),
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
        return Paths.line(shoot2Part2Start, shoot2Part2).linear(shoot2Part2Start, shoot2Part2);
    }

    public Path collectFlowerPart1() {
        return Paths.curve(collectFlowerPart1Start, collectFlowerPart1Control1, collectFlowerPart1).linear(collectFlowerPart1Start, collectFlowerPart1);
    }

    public Path shoot3Part1() {
        return Paths.line(shoot3Part1Start, shoot3Part1).linear(shoot3Part1Start, shoot3Part1);
    }

    public Path shoot3Part2() {
        return Paths.line(shoot3Part2Start, shoot3Part2).linear(shoot3Part2Start, shoot3Part2);
    }

    public Path park() {
        return Paths.line(parkStart, park).linear(parkStart, park);
    }
}
