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

    private final Pose start = poseFactory.of(60, 133, 270);
    private final Pose startshoot = poseFactory.of(58, 117, 270);
    private final Pose point2 = poseFactory.of(47.2639, 125.9349, 90);
    private final Pose point2Control1 = poseFactory.of(46.9362, 117.7028, 0);
    private final Pose point3Start = poseFactory.of(47.2639, 125.9349, 90);
    private final Pose point3 = poseFactory.of(58, 117, 270);
    private final Pose point3Control1 = poseFactory.of(47.2171, 117.6859, 0);
    private final Pose point4 = poseFactory.of(12.2585, 109.0543, 270);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, startshoot()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4())
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

    public Path startshoot() {
        return line(start, startshoot).constant(startshoot);
    }

    public Path path2() {
        return curve(startshoot, point2Control1, point2).tangent();
    }

    public Path path3() {
        return curve(point3Start, point3Control1, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return line(point3, point4).constant(point4);
    }
}
