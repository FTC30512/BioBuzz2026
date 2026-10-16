package org.firstinspires.ftc.teamcode;

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

@Autonomous(name = "Red2_BothSides2Flowers", group = "Red2")
public class Red2_BothSides2Flowers extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59, 133, 90);
    private final Pose shoot1Start = poseFactory.of(59, 133, 270);
    private final Pose shoot1 = poseFactory.of(59, 108.2133, 270);
    private final Pose collectFlower1Part1 = poseFactory.of(47.5363, 123.3834, 90);
    private final Pose collectFlower1Part1Control1 = poseFactory.of(46.2971, 117.7427, 0);
    private final Pose collectFlower1Part2 = poseFactory.of(47.1598, 133.4758, 90);
    private final Pose shoot2Part1 = poseFactory.of(47.5138, 123.3653, 90);
    private final Pose shoot2Part2 = poseFactory.of(58.7556, 108.4637, 270);
    private final Pose shoot2Part2Control1 = poseFactory.of(46.8532, 117.0095, 0);
    private final Pose collectFlower2Part1 = poseFactory.of(23.6192, 68.6632, 270);
    private final Pose collectFlower2Part1Control1 = poseFactory.of(22.013, 105.4335, 0);
    private final Pose collectFlower2Part2 = poseFactory.of(8.9257, 47.2098, 180);
    private final Pose collectFlower2Part2Control1 = poseFactory.of(20.8951, 47.1911, 0);
    private final Pose shoot3 = poseFactory.of(58.4413, 33.3497, 90);
    private final Pose shoot3Control1 = poseFactory.of(47.6287, 32.3843, 0);
    private final Pose park = poseFactory.of(10.4741, 95.9128, 90);
    private final Pose parkControl1 = poseFactory.of(19.3472, 35.8005, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot1()),
                follow(follower, collectFlower1Part1()),
                follow(follower, collectFlower1Part2()),
                follow(follower, shoot2Part1()),
                follow(follower, shoot2Part2()),
                follow(follower, collectFlower2Part1()),
                follow(follower, collectFlower2Part2()),
                follow(follower, shoot3()),
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
        return line(shoot1Start, shoot1).linear(shoot1Start, shoot1);
    }

    public Path collectFlower1Part1() {
        return curve(shoot1, collectFlower1Part1Control1, collectFlower1Part1).linear(shoot1, collectFlower1Part1);
    }

    public Path collectFlower1Part2() {
        return line(collectFlower1Part1, collectFlower1Part2).linear(collectFlower1Part1, collectFlower1Part2);
    }

    public Path shoot2Part1() {
        return line(collectFlower1Part2, shoot2Part1).linear(collectFlower1Part2, shoot2Part1);
    }

    public Path shoot2Part2() {
        return curve(shoot2Part1, shoot2Part2Control1, shoot2Part2).linear(shoot2Part1, shoot2Part2);
    }

    public Path collectFlower2Part1() {
        return curve(shoot2Part2, collectFlower2Part1Control1, collectFlower2Part1).linear(shoot2Part2, collectFlower2Part1);
    }

    public Path collectFlower2Part2() {
        return curve(collectFlower2Part1, collectFlower2Part2Control1, collectFlower2Part2).linear(collectFlower2Part1, collectFlower2Part2);
    }

    public Path shoot3() {
        return curve(collectFlower2Part2, shoot3Control1, shoot3).linear(collectFlower2Part2, shoot3);
    }

    public Path park() {
        return curve(shoot3, parkControl1, park).linear(shoot3, park);
    }
}
