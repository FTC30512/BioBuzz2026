package org.firstinspires.ftc.teamcode.pedro.Autonomous.Red1;

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

@Autonomous(name = "Red1_BothSidesWithout2ndFlower", group = "Red1")
public class Red1_BothSidesWithout2ndFlower extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59, 8, 90);
    private final Pose startshootpos1 = poseFactory.of(59, 24, 90);
    private final Pose shootpos1collectgarden1 = poseFactory.of(21, 8, 180);
    private final Pose shootpos1collectgarden1Control1 = poseFactory.of(57.5692, 17.2571, 0);
    private final Pose collectgarden1collectgarden2 = poseFactory.of(8, 7.8, 180);
    private final Pose collectgarden2shootpos2 = poseFactory.of(59, 118, 270);
    private final Pose collectgarden2shootpos2Control1 = poseFactory.of(86.5916, 17.5543, 0);
    private final Pose shootpos2endpos = poseFactory.of(10, 96, 270);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, startshootpos1()),
                follow(follower, shootpos1collectgarden1()),
                follow(follower, collectgarden1collectgarden2()),
                follow(follower, collectgarden2shootpos2()),
                follow(follower, shootpos2endpos())
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

    public Path startshootpos1() {
        return line(start, startshootpos1).linear(start, startshootpos1);
    }

    public Path shootpos1collectgarden1() {
        return curve(startshootpos1, shootpos1collectgarden1Control1, shootpos1collectgarden1).linear(startshootpos1, shootpos1collectgarden1);
    }

    public Path collectgarden1collectgarden2() {
        return line(shootpos1collectgarden1, collectgarden1collectgarden2).linear(shootpos1collectgarden1, collectgarden1collectgarden2);
    }

    public Path collectgarden2shootpos2() {
        return curve(collectgarden1collectgarden2, collectgarden2shootpos2Control1, collectgarden2shootpos2).linear(collectgarden1collectgarden2, collectgarden2shootpos2);
    }

    public Path shootpos2endpos() {
        return line(collectgarden2shootpos2, shootpos2endpos).constant(shootpos2endpos);
    }
}
