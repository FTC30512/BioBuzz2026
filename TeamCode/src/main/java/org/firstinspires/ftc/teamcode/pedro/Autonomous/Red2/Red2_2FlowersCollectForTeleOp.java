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

@Autonomous(name = "Red2_2FlowersCollectForTeleOp", group = "Red2")
public class Red2_2FlowersCollectForTeleOp extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59.477, 132.0158, 270);
    private final Pose shoot1 = poseFactory.of(59.477, 123.421, 270);
    private final Pose approachFlower = poseFactory.of(46.9871, 119.9899, 90);
    private final Pose approachFlowerControl1 = poseFactory.of(57.033, 115.6379, 0);
    private final Pose collectPollen = poseFactory.of(46.7586, 132.2773, 90);
    private final Pose approachShoot = poseFactory.of(46.6466, 121.7284, 90);
    private final Pose shoot2 = poseFactory.of(50.4727, 114.9569, 270);
    private final Pose approachFlower_2 = poseFactory.of(17.3922, 47.658, 180);
    private final Pose approachFlower_2Control1 = poseFactory.of(25.25, 95.6925, 0);
    private final Pose collectPollen_2 = poseFactory.of(9.7342, 47.5718, 180);
    private final Pose approachPark = poseFactory.of(17.0316, 47.7917, 180);
    private final Pose park = poseFactory.of(8.658, 114.0891, 0);
    private final Pose parkControl1 = poseFactory.of(43.1092, 86.375, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot1()),
                follow(follower, approachFlower()),
                follow(follower, collectPollen()),
                follow(follower, approachShoot()),
                follow(follower, shoot2()),
                follow(follower, approachFlower_2()),
                follow(follower, collectPollen_2()),
                follow(follower, approachPark()),
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

    public Path approachShoot() {
        return line(collectPollen, approachShoot).linear(collectPollen, approachShoot);
    }

    public Path shoot2() {
        return line(approachShoot, shoot2).linear(approachShoot, shoot2);
    }

    public Path approachFlower_2() {
        return curve(shoot2, approachFlower_2Control1, approachFlower_2).linear(shoot2, approachFlower_2);
    }

    public Path collectPollen_2() {
        return line(approachFlower_2, collectPollen_2).linear(approachFlower_2, collectPollen_2);
    }

    public Path approachPark() {
        return line(collectPollen_2, approachPark).linear(collectPollen_2, approachPark);
    }

    public Path park() {
        return curve(approachPark, parkControl1, park).linear(approachPark, park);
    }
}
