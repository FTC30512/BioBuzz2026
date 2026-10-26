package org.firstinspires.ftc.teamcode.pedro.Autonomous.Red1;

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

@Autonomous(name = "Red1_BothFlowers", group = "Red1")
public class Red1_BothFlowers extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorY(70.75);

    private final Pose start = poseFactory.of(59, 133, -90);
    private final Pose shoot1 = poseFactory.of(57.5, 123, -90);
    private final Pose collectHomeFlowerStart = poseFactory.of(57.5, 123, -90);
    private final Pose collectHomeFlower = poseFactory.of(15.5, 94, 180);
    private final Pose collectHomeFlowerControl1 = poseFactory.of(39.5, 99, 0);
    private final Pose shoot2Part1Start = poseFactory.of(15.5, 94, 180);
    private final Pose shoot2Part1 = poseFactory.of(23.5, 65, 90);
    private final Pose shoot2Part1Control1 = poseFactory.of(23.5, 90, 0);
    private final Pose shoot2Part2Start = poseFactory.of(23.5, 65, 90);
    private final Pose shoot2Part2 = poseFactory.of(57.5, 35, 90);
    private final Pose shoot2Part2Control1 = poseFactory.of(23.5, 36, 0);
    private final Pose collectAwayFlowerStart = poseFactory.of(57.5, 35, -90);
    private final Pose collectAwayFlower = poseFactory.of(47.5, 15, -90);
    private final Pose collectAwayFlowerControl1 = poseFactory.of(47.5, 26, 0);
    private final Pose shoot3Part1Start = poseFactory.of(47.5, 15, -90);
    private final Pose shoot3Part1 = poseFactory.of(47.5, 21, -90);
    private final Pose shoot3Part2Start = poseFactory.of(47.5, 21, 90);
    private final Pose shoot3Part2 = poseFactory.of(57.5, 35, 90);
    private final Pose shoot3Part2Control1 = poseFactory.of(48.5, 33, 0);
    private final Pose parkStart = poseFactory.of(57.5, 35, 90);
    private final Pose park = poseFactory.of(11.5, 46, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot1()),
                follow(follower, collectHomeFlower()),
                follow(follower, shoot2Part1()),
                follow(follower, shoot2Part2()),
                follow(follower, collectAwayFlower()),
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

    public Path collectHomeFlower() {
        return Paths.curve(collectHomeFlowerStart, collectHomeFlowerControl1, collectHomeFlower).linear(collectHomeFlowerStart, collectHomeFlower);
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
        return Paths.line(shoot3Part1Start, shoot3Part1).linear(shoot3Part1Start, shoot3Part1);
    }

    public Path shoot3Part2() {
        return Paths.curve(shoot3Part2Start, shoot3Part2Control1, shoot3Part2).linear(shoot3Part2Start, shoot3Part2);
    }

    public Path park() {
        return Paths.line(parkStart, park).linear(parkStart, park);
    }
}
