package org.firstinspires.ftc.teamcode.pedro.Autonomous.Red2;

import static com.pedropathing.api.Paths.*;

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

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Red2_1FlowerHomeSide", group = "Red2")
public class Red2_1FlowerHomeSide extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59.9102, 133.3705, 90);
    private final Pose shoot1Start = poseFactory.of(59.9102, 133.3705, 270);
    private final Pose shoot1 = poseFactory.of(59, 120, 270);
    private final Pose collectFlowerStart = poseFactory.of(59, 120, 90);
    private final Pose collectFlower = poseFactory.of(47, 129, 90);
    private final Pose collectFlowerControl1 = poseFactory.of(47, 120, 0);
    private final Pose shoot2 = poseFactory.of(59, 120, 90);
    private final Pose shoot2Control1 = poseFactory.of(47, 120, 0);
    private final Pose parkStart = poseFactory.of(59, 120, 270);
    private final Pose park = poseFactory.of(11, 117, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot1()),
                follow(follower, collectFlower()),
                follow(follower, shoot2()),
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

    public Path collectFlower() {
        return curve(collectFlowerStart, collectFlowerControl1, collectFlower).linear(collectFlowerStart, collectFlower);
    }

    public Path shoot2() {
        return curve(collectFlower, shoot2Control1, shoot2).linear(collectFlower, shoot2);
    }

    public Path park() {
        return line(parkStart, park).linear(parkStart, park);
    }
}
