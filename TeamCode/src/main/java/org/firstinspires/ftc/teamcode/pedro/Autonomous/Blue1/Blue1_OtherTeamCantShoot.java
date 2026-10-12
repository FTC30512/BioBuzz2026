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

@Autonomous(name = "Blue_OtherTeamCantShoot", group = "Blue1")
public class Blue1_OtherTeamCantShoot extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(84.3489, 132.8817, 270);
    private final Pose shoot1 = poseFactory.of(84.1045, 117.6252, 270);
    private final Pose point2 = poseFactory.of(121.6054, 133.1693, 0);
    private final Pose point2Control1 = poseFactory.of(93.4689, 127.4283, 0);
    private final Pose point3 = poseFactory.of(133.1019, 133.0691, 0);
    private final Pose point4 = poseFactory.of(82.9499, 94.1917, 90);
    private final Pose point4Control1 = poseFactory.of(90.1494, 125.7677, 0);
    private final Pose point5 = poseFactory.of(82.7081, 34.4991, 90);
    private final Pose point6 = poseFactory.of(94.5259, 17.2401, 270);
    private final Pose point6Control1 = poseFactory.of(95.3713, 29.3722, 0);
    private final Pose point7 = poseFactory.of(94.4836, 8.9508, 270);
    private final Pose point8 = poseFactory.of(94.5984, 17.3722, 270);
    private final Pose point9 = poseFactory.of(97.5086, 34.4715, 130);
    private final Pose point10 = poseFactory.of(131.2142, 26.6813, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6()),
                follow(follower, path7()),
                follow(follower, path8()),
                follow(follower, path9()),
                follow(follower, path10())
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

    public Path path2() {
        return curve(shoot1, point2Control1, point2).linear(shoot1, point2);
    }

    public Path path3() {
        return line(point2, point3).linear(point2, point3);
    }

    public Path path4() {
        return curve(point3, point4Control1, point4).linear(point3, point4);
    }

    public Path path5() {
        return line(point4, point5).linear(point4, point5);
    }

    public Path path6() {
        return curve(point5, point6Control1, point6).linear(point5, point6);
    }

    public Path path7() {
        return line(point6, point7).linear(point6, point7);
    }

    public Path path8() {
        return line(point7, point8).linear(point7, point8);
    }

    public Path path9() {
        return line(point8, point9).linear(point8, point9);
    }

    public Path path10() {
        return line(point9, point10).linear(point9, point10);
    }
}
