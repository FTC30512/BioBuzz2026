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

@Autonomous(name = "Blue2_BothFlowers", group = "Blue2")
public class Blue2_BothFlowers extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(81.6606, 8.4888, 90);
    private final Pose shoot1 = poseFactory.of(81.905, 23.0475, 90);
    private final Pose collectHomeFlowerPart1 = poseFactory.of(94.3333, 18.1091, 270);
    private final Pose collectHomeFlowerPart1Control1 = poseFactory.of(93.025, 22.9905, 0);
    private final Pose collectHomeFlowerPart2 = poseFactory.of(93.8377, 8.8212, 270);
    private final Pose shoot2Part1 = poseFactory.of(93.9301, 17.9706, 270);
    private final Pose shoot2Part2 = poseFactory.of(82.9499, 23.12, 90);
    private final Pose shoot2Part2Control1 = poseFactory.of(85.9326, 19.9007, 0);
    private final Pose collectOtherFlowerPart1 = poseFactory.of(115.3463, 69.0276, 45);
    private final Pose collectOtherFlowerPart1Control1 = poseFactory.of(118.3886, 47.0527, 0);
    private final Pose collectOtherFlowerPart2 = poseFactory.of(133.0691, 94.2245, 0);
    private final Pose collectOtherFlowerPart2Control1 = poseFactory.of(118.3437, 93.0345, 0);
    private final Pose shoot3 = poseFactory.of(83.3342, 108.785, 270);
    private final Pose shoot3Control1 = poseFactory.of(115.0492, 108.2168, 0);
    private final Pose park = poseFactory.of(131.2358, 45.9692, 180);
    private final Pose parkControl1 = poseFactory.of(133.6675, 87.4611, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot1()),
                follow(follower, collectHomeFlowerPart1()),
                follow(follower, collectHomeFlowerPart2()),
                follow(follower, shoot2Part1()),
                follow(follower, shoot2Part2()),
                follow(follower, collectOtherFlowerPart1()),
                follow(follower, collectOtherFlowerPart2()),
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
        return line(start, shoot1).linear(start, shoot1);
    }

    public Path collectHomeFlowerPart1() {
        return curve(shoot1, collectHomeFlowerPart1Control1, collectHomeFlowerPart1).linear(shoot1, collectHomeFlowerPart1);
    }

    public Path collectHomeFlowerPart2() {
        return line(collectHomeFlowerPart1, collectHomeFlowerPart2).linear(collectHomeFlowerPart1, collectHomeFlowerPart2);
    }

    public Path shoot2Part1() {
        return line(collectHomeFlowerPart2, shoot2Part1).linear(collectHomeFlowerPart2, shoot2Part1);
    }

    public Path shoot2Part2() {
        return curve(shoot2Part1, shoot2Part2Control1, shoot2Part2).linear(shoot2Part1, shoot2Part2);
    }

    public Path collectOtherFlowerPart1() {
        return curve(shoot2Part2, collectOtherFlowerPart1Control1, collectOtherFlowerPart1).linear(shoot2Part2, collectOtherFlowerPart1);
    }

    public Path collectOtherFlowerPart2() {
        return curve(collectOtherFlowerPart1, collectOtherFlowerPart2Control1, collectOtherFlowerPart2).linear(collectOtherFlowerPart1, collectOtherFlowerPart2);
    }

    public Path shoot3() {
        return curve(collectOtherFlowerPart2, shoot3Control1, shoot3).linear(collectOtherFlowerPart2, shoot3);
    }

    public Path park() {
        return curve(shoot3, parkControl1, park).linear(shoot3, park);
    }
}
