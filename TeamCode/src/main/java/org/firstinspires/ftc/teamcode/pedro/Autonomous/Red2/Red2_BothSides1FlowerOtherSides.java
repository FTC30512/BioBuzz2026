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

@Autonomous(name = "Red2_BothSides1FlowerOtherSides", group = "Red2")
public class Red2_BothSides1FlowerOtherSides extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59, 132, 90);
    private final Pose shoot1Start = poseFactory.of(59.477, 132.0158, -90);
    private final Pose shoot1 = poseFactory.of(59.477, 123.421, -90);
    private final Pose approachFlowerStart = poseFactory.of(59.477, 123.421, 270);
    private final Pose approachFlower = poseFactory.of(28.1695, 74.3405, 270);
    private final Pose approachFlowerControl1 = poseFactory.of(32.3764, 114.8161, 0);
    private final Pose collectPollen = poseFactory.of(7.5223, 47.4462, 180);
    private final Pose collectPollenControl1 = poseFactory.of(23.3434, 48.0761, 0);
    private final Pose shoot2 = poseFactory.of(58.3484, 32.9438, 90);
    private final Pose shoot2Control1 = poseFactory.of(28.4483, 34.7457, 0);
    private final Pose park = poseFactory.of(10.4698, 97.1782, 180);
    private final Pose parkControl1 = poseFactory.of(19.625, 39.7687, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot1()),
                follow(follower, path2()),
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

    public Path path2() {
        return path(curve(approachFlowerStart, approachFlowerControl1, approachFlower).linear(approachFlowerStart, approachFlower), curve(approachFlower, collectPollenControl1, collectPollen).linear(approachFlower, collectPollen));
    }

    public Path shoot2() {
        return curve(collectPollen, shoot2Control1, shoot2).linear(collectPollen, shoot2);
    }

    public Path park() {
        return curve(shoot2, parkControl1, park).linear(shoot2, park);
    }
}
