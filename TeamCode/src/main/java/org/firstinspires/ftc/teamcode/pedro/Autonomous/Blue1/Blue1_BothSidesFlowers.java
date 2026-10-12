package org.firstinspires.ftc.teamcode.pedro.Autonomous.Blue1;

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


@Autonomous(name = "Blue1_BothSidesFlowers", group = "Blue1")
public class Blue1_BothSidesFlowers extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(82.023, 132.0158, 90);
    private final Pose shoot1 = poseFactory.of(82.023, 123.421, 270);
    private final Pose collectPollen = poseFactory.of(133.7067, 95.0195, 0);
    private final Pose collectPollenControl1 = poseFactory.of(101.8922, 99.3089, 0);
    private final Pose approachShoot = poseFactory.of(118.1652, 64.9325, 90);
    private final Pose approachShootControl1 = poseFactory.of(118.2555, 89.598, 0);
    private final Pose shoot2 = poseFactory.of(83.3621, 28.5029, 90);
    private final Pose shoot2Control1 = poseFactory.of(118.3103, 26.944, 0);
    private final Pose collectPollen_2 = poseFactory.of(94.6494, 10.4799, 270);
    private final Pose approachShoot_2 = poseFactory.of(95.9124, 19.8118, 270);
    private final Pose shoot3 = poseFactory.of(98.8147, 27.658, 115);
    private final Pose park = poseFactory.of(130.7249, 42.9735, 0);
    private final Pose parkControl1 = poseFactory.of(112.4971, 42.1739, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot1()),
                follow(follower, collectPollen()),
                follow(follower, approachShoot()),
                follow(follower, shoot2()),
                follow(follower, collectPollen_2()),
                follow(follower, approachShoot_2()),
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

    public Path collectPollen() {
        return curve(shoot1, collectPollenControl1, collectPollen).linear(shoot1, collectPollen);
    }

    public Path approachShoot() {
        return curve(collectPollen, approachShootControl1, approachShoot).linear(collectPollen, approachShoot);
    }

    public Path shoot2() {
        return curve(approachShoot, shoot2Control1, shoot2).linear(approachShoot, shoot2);
    }

    public Path collectPollen_2() {
        return line(shoot2, collectPollen_2).linear(shoot2, collectPollen_2);
    }

    public Path approachShoot_2() {
        return line(collectPollen_2, approachShoot_2).linear(collectPollen_2, approachShoot_2);
    }

    public Path shoot3() {
        return line(approachShoot_2, shoot3).linear(approachShoot_2, shoot3);
    }

    public Path park() {
        return curve(shoot3, parkControl1, park).linear(shoot3, park);
    }
}
