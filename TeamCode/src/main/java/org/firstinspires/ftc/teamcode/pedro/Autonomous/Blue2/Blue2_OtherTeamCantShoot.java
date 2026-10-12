package org.firstinspires.ftc.teamcode.pedro.Autonomous.Blue2;

import static com.pedropathing.api.Paths.*;

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

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Blue2_OtherTeamCantShoot", group = "Blue2")
public class Blue2_OtherTeamCantShoot extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(81.6606, 8.4888, 90);
    private final Pose shoot1 = poseFactory.of(81.9318, 17.6282, 90);
    private final Pose approachFlower = poseFactory.of(122.9993, 94.2063, 0);
    private final Pose approachFlowerControl1 = poseFactory.of(127.472, 44.5999, 0);
    private final Pose collectPollen = poseFactory.of(132.3513, 94.2063, 0);
    private final Pose shootHalf2 = poseFactory.of(82.056, 116.4957, 270);
    private final Pose shootHalf2Control1 = poseFactory.of(96.2759, 100.4397, 0);
    private final Pose shootHalf3Start = poseFactory.of(82.056, 116.4957, 90);
    private final Pose shootHalf3 = poseFactory.of(81.648, 26.931, 90);
    private final Pose approachFlower_2 = poseFactory.of(94.704, 21.0517, 270);
    private final Pose approachFlower_2Control1 = poseFactory.of(91.4526, 26.0977, 0);
    private final Pose collectPollen_2 = poseFactory.of(94.8908, 9.7328, 270);
    private final Pose shoot4 = poseFactory.of(96.6164, 18.9971, 270);
    private final Pose parkStart = poseFactory.of(96.6164, 18.9971, 115);
    private final Pose park = poseFactory.of(131.7716, 25.4052, 180);
    private final Pose parkControl1 = poseFactory.of(110.0575, 28.5575, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot1()),
                follow(follower, approachFlower()),
                follow(follower, collectPollen()),
                follow(follower, shootHalf2()),
                follow(follower, shootHalf3()),
                follow(follower, approachFlower_2()),
                follow(follower, collectPollen_2()),
                follow(follower, shoot4()),
                follow(follower, park())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
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

    public Path approachFlower() {
        return curve(shoot1, approachFlowerControl1, approachFlower).linear(shoot1, approachFlower);
    }

    public Path collectPollen() {
        return line(approachFlower, collectPollen).linear(approachFlower, collectPollen);
    }

    public Path shootHalf2() {
        return curve(collectPollen, shootHalf2Control1, shootHalf2).linear(collectPollen, shootHalf2);
    }

    public Path shootHalf3() {
        return line(shootHalf3Start, shootHalf3).linear(shootHalf3Start, shootHalf3);
    }

    public Path approachFlower_2() {
        return curve(shootHalf3, approachFlower_2Control1, approachFlower_2).linear(shootHalf3, approachFlower_2);
    }

    public Path collectPollen_2() {
        return line(approachFlower_2, collectPollen_2).linear(approachFlower_2, collectPollen_2);
    }

    public Path shoot4() {
        return line(collectPollen_2, shoot4).linear(collectPollen_2, shoot4);
    }

    public Path park() {
        return curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
}
