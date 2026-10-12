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

@Autonomous(name = "Red1_OtherTeamCantShoot", group = "Red1")
public class Red1_OtherTeamCantShoot extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59, 8, 90);
    private final Pose startshootpos1 = poseFactory.of(59, 24, 90);
    private final Pose shootpos1collectgarden1 = poseFactory.of(21, 8, 180);
    private final Pose shootpos1collectgarden1Control1 = poseFactory.of(57.5692, 17.2571, 0);
    private final Pose collectgarden1collectgarden2 = poseFactory.of(8, 7.8, 180);
    private final Pose collectgarden2otherside1 = poseFactory.of(59.7332, 42.0846, 270);
    private final Pose collectgarden2otherside1Control1 = poseFactory.of(44.3284, 17.6219, 0);
    private final Pose otherside1otherside2 = poseFactory.of(59.0648, 122.6131, 270);
    private final Pose otherside2collectotherflower1 = poseFactory.of(47.1667, 124.173, 90);
    private final Pose collectotherflower1collectotherflower2 = poseFactory.of(46.9801, 132.6926, 90);
    private final Pose collectotherflower2collectotherflower1 = poseFactory.of(47.5363, 124.5682, 90);
    private final Pose collectotherflower1shootpos2 = poseFactory.of(58.8204, 122.7254, 270);
    private final Pose shootpos2 = poseFactory.of(10.0009, 115.2763, 180);
    private final Pose shootpos2Control1 = poseFactory.of(36.5613, 114.1269, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, startshootpos1()),
                follow(follower, shootpos1collectgarden1()),
                follow(follower, collectgarden1collectgarden2()),
                follow(follower, collectgarden2otherside1()),
                follow(follower, otherside1otherside2()),
                follow(follower, otherside2collectotherflower1()),
                follow(follower, collectotherflower1collectotherflower2()),
                follow(follower, collectotherflower2collectotherflower1()),
                follow(follower, collectotherflower1shootpos2()),
                follow(follower, shootpos2())
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

    public Path collectgarden2otherside1() {
        return curve(collectgarden1collectgarden2, collectgarden2otherside1Control1, collectgarden2otherside1).linear(collectgarden1collectgarden2, collectgarden2otherside1);
    }

    public Path otherside1otherside2() {
        return line(collectgarden2otherside1, otherside1otherside2).linear(collectgarden2otherside1, otherside1otherside2);
    }

    public Path otherside2collectotherflower1() {
        return line(otherside1otherside2, otherside2collectotherflower1).linear(otherside1otherside2, otherside2collectotherflower1);
    }

    public Path collectotherflower1collectotherflower2() {
        return line(otherside2collectotherflower1, collectotherflower1collectotherflower2).linear(otherside2collectotherflower1, collectotherflower1collectotherflower2);
    }

    public Path collectotherflower2collectotherflower1() {
        return line(collectotherflower1collectotherflower2, collectotherflower2collectotherflower1).linear(collectotherflower1collectotherflower2, collectotherflower2collectotherflower1);
    }

    public Path collectotherflower1shootpos2() {
        return line(collectotherflower2collectotherflower1, collectotherflower1shootpos2).linear(collectotherflower2collectotherflower1, collectotherflower1shootpos2);
    }

    public Path shootpos2() {
        return curve(collectotherflower1shootpos2, shootpos2Control1, shootpos2).linear(collectotherflower1shootpos2, shootpos2);
    }
}
