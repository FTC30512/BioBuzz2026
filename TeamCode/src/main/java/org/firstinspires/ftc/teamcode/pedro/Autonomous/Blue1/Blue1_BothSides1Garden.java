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

@Autonomous(name = "Blue1BothSides1Garden", group = "Blue1")
public class Blue1_BothSides1Garden extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(85, 132, 270);
    private final Pose shoot1 = poseFactory.of(85, 123, 270);
    private final Pose approachGarden = poseFactory.of(122.6825, 120.694, 5.5163);
    private final Pose approachGardenControl1 = poseFactory.of(91.6394, 117.6624, 0);
    private final Pose collectPollen = poseFactory.of(133.1645, 133.2408, 92.234);
    private final Pose collectPollenControl1 = poseFactory.of(133.6078, 123.2989, 0);
    private final Pose shoot2Start = poseFactory.of(133.1645, 133.2408, 90);
    private final Pose shoot2 = poseFactory.of(83.3621, 28.5029, 90);
    private final Pose shoot2Control1 = poseFactory.of(118.3103, 26.944, 0);
    private final Pose parkStart = poseFactory.of(83.3621, 28.5029, 0);
    private final Pose park = poseFactory.of(130.7249, 42.9735, 0);
    private final Pose parkControl1 = poseFactory.of(112.4971, 42.1739, 0);

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
        return line(start, shoot1).linear(start, shoot1);
    }

    public Path path2() {
        return path(curve(shoot1, approachGardenControl1, approachGarden).tangent(), curve(approachGarden, collectPollenControl1, collectPollen).tangent());
    }

    public Path shoot2() {
        return curve(shoot2Start, shoot2Control1, shoot2).linear(shoot2Start, shoot2);
    }

    public Path park() {
        return curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
}
