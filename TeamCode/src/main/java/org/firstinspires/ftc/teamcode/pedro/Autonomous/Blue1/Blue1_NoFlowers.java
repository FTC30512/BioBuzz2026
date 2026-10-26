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

@Autonomous(name = "Blue1_NoFlowers", group = "Blue1")
public class Blue1_NoFlowers extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(82.5, 133, 270);
    private final Pose shoot1 = poseFactory.of(84, 123, 270);
    private final Pose collectGardenPart1Start = poseFactory.of(84, 123, 0);
    private final Pose collectGardenPart1 = poseFactory.of(120, 132, 0);
    private final Pose collectGardenPart2Start = poseFactory.of(120, 132, 0);
    private final Pose collectGardenPart2 = poseFactory.of(132, 132, 0);
    private final Pose shoot2Part1Start = poseFactory.of(132, 132, 0);
    private final Pose shoot2Part1 = poseFactory.of(83, 94, 90);
    private final Pose shoot2Part1Control1 = poseFactory.of(90, 125, 0);
    private final Pose shoot2Part2Start = poseFactory.of(83, 94, 90);
    private final Pose shoot2Part2 = poseFactory.of(83, 35, 90);
    private final Pose parkStart = poseFactory.of(83, 35, 90);
    private final Pose park = poseFactory.of(130, 46, 180);
    private final Pose parkControl1 = poseFactory.of(120, 36, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot1()),
                follow(follower, collectGardenPart1()),
                follow(follower, collectGardenPart2()),
                follow(follower, shoot2Part1()),
                follow(follower, shoot2Part2()),
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
        return Paths.line(collectGardenPart1Start, collectGardenPart1).linear(collectGardenPart1Start, collectGardenPart1);
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

    public Path park() {
        return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
}
