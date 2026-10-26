package org.firstinspires.ftc.teamcode.pedro.Autonomous.Red2;

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

@Autonomous(name = "Red2_1FlowerHomeSide", group = "Red2")
public class Red2_1FlowerHomeSide extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59, 132, 270);
    private final Pose shoot1 = poseFactory.of(59, 117, 270);
    private final Pose collectFlowerPart1 = poseFactory.of(47, 124.7, 90);
    private final Pose collectFlowerPart1Control1 = poseFactory.of(50.4, 120, 0);
    private final Pose collectFlowerPart2 = poseFactory.of(47, 133, 90);
    private final Pose shoot2Part1 = poseFactory.of(47, 124.6, 90);
    private final Pose shoot2Part2Start = poseFactory.of(47, 124.6, 90);
    private final Pose shoot2Part2 = poseFactory.of(60, 117, 270);
    private final Pose shoot2Part2Control1 = poseFactory.of(50, 119.6, 0);
    private final Pose park = poseFactory.of(9.7, 117, 180);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot1()),
                follow(follower, collectFlowerPart1()),
                follow(follower, collectFlowerPart2()),
                follow(follower, shoot2Part1()),
                follow(follower, shoot2Part2()),
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

    public Path collectFlowerPart1() {
        return curve(shoot1, collectFlowerPart1Control1, collectFlowerPart1).linear(shoot1, collectFlowerPart1);
    }

    public Path collectFlowerPart2() {
        return line(collectFlowerPart1, collectFlowerPart2).linear(collectFlowerPart1, collectFlowerPart2);
    }

    public Path shoot2Part1() {
        return line(collectFlowerPart2, shoot2Part1).reverseTangent();
    }

    public Path shoot2Part2() {
        return curve(shoot2Part2Start, shoot2Part2Control1, shoot2Part2).linear(shoot2Part2Start, shoot2Part2);
    }

    public Path park() {
        return line(shoot2Part2, park).linear(shoot2Part2, park);
    }
}
