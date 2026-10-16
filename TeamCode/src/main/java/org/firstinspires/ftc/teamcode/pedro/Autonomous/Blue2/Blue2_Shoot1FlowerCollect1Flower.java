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

@Autonomous(name = "Blue2_Shoot1FlowerCollect1Flower", group = "Blue2")
public class Blue2_Shoot1FlowerCollect1Flower extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(81, 8, 90);
    private final Pose shoot1 = poseFactory.of(81, 17, 90);
    private final Pose approachFlower = poseFactory.of(94.2529, 25.3491, 270);
    private final Pose collectPollen = poseFactory.of(94.1537, 8.4698, 270);
    private final Pose approachShoot = poseFactory.of(94.0345, 18.8879, 270);
    private final Pose shoot2 = poseFactory.of(84.778, 26.167, 90);
    private final Pose approachFlower_2 = poseFactory.of(116.4935, 94.2063, 0);
    private final Pose approachFlower_2Control1 = poseFactory.of(127.472, 44.5999, 0);
    private final Pose collectPollen_2 = poseFactory.of(132.3513, 94.2063, 0);
    private final Pose park = poseFactory.of(132.8822, 44.5259, 180);
    private final Pose parkControl1 = poseFactory.of(102.7816, 70.3506, 0);

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
        return line(shoot1, approachFlower).linear(shoot1, approachFlower);
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

    public Path park() {
        return curve(collectPollen_2, parkControl1, park).linear(collectPollen_2, park);
    }
}
