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
public class Red1_OtherTeamCantShootLong extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59, 8, 90);
    private final Pose shoot1 = poseFactory.of(58.9326, 24.5138, 90);
    private final Pose collectGardenPart1 = poseFactory.of(20.6563, 8.6589, 180);
    private final Pose collectGardenPart1Control1 = poseFactory.of(58.0751, 12.4093, 0);
    private final Pose collectGardenPart2 = poseFactory.of(8.3869, 8.2573, 180);
    private final Pose shoot2Part1 = poseFactory.of(59.5285, 46.4067, 270);
    private final Pose shoot2Part1Control1 = poseFactory.of(47.5332, 18.5978, 0);
    private final Pose shoot2Part2 = poseFactory.of(59.2617, 108.7979, 270);
    private final Pose collectHomeFlowerPart1 = poseFactory.of(25.8307, 73.905, 270);
    private final Pose collectHomeFlowerPart1Control1 = poseFactory.of(28.9007, 106.6952, 0);
    private final Pose collectHomeFlowerPart2 = poseFactory.of(8.5095, 47.671, 180);
    private final Pose collectHomeFlowerPart2Control1 = poseFactory.of(23.6088, 48.3143, 0);
    private final Pose shoot3Part1 = poseFactory.of(26.1598, 73.4465, 270);
    private final Pose shoot3Part1Control1 = poseFactory.of(23.0406, 48.1321, 0);
    private final Pose shoot3Part2 = poseFactory.of(41.2547, 107.152, 270);
    private final Pose shoot3Part2Control1 = poseFactory.of(28.3169, 106.9948, 0);
    private final Pose park = poseFactory.of(10.1425, 95.5786, 180);
    private final Pose parkControl1 = poseFactory.of(23.7962, 105.9326, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot1()),
                follow(follower, collectGardenPart1()),
                follow(follower, collectGardenPart2()),
                follow(follower, shoot2Part1()),
                follow(follower, shoot2Part2()),
                follow(follower, collectHomeFlowerPart1()),
                follow(follower, collectHomeFlowerPart2()),
                follow(follower, shoot3Part1()),
                follow(follower, shoot3Part2()),
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

    public Path collectGardenPart1() {
        return curve(shoot1, collectGardenPart1Control1, collectGardenPart1).linear(shoot1, collectGardenPart1);
    }

    public Path collectGardenPart2() {
        return line(collectGardenPart1, collectGardenPart2).linear(collectGardenPart1, collectGardenPart2);
    }

    public Path shoot2Part1() {
        return curve(collectGardenPart2, shoot2Part1Control1, shoot2Part1).linear(collectGardenPart2, shoot2Part1);
    }

    public Path shoot2Part2() {
        return line(shoot2Part1, shoot2Part2).linear(shoot2Part1, shoot2Part2);
    }

    public Path collectHomeFlowerPart1() {
        return curve(shoot2Part2, collectHomeFlowerPart1Control1, collectHomeFlowerPart1).linear(shoot2Part2, collectHomeFlowerPart1);
    }

    public Path collectHomeFlowerPart2() {
        return curve(collectHomeFlowerPart1, collectHomeFlowerPart2Control1, collectHomeFlowerPart2).linear(collectHomeFlowerPart1, collectHomeFlowerPart2);
    }

    public Path shoot3Part1() {
        return curve(collectHomeFlowerPart2, shoot3Part1Control1, shoot3Part1).linear(collectHomeFlowerPart2, shoot3Part1);
    }

    public Path shoot3Part2() {
        return curve(shoot3Part1, shoot3Part2Control1, shoot3Part2).linear(shoot3Part1, shoot3Part2);
    }

    public Path park() {
        return curve(shoot3Part2, parkControl1, park).linear(shoot3Part2, park);
    }
}
