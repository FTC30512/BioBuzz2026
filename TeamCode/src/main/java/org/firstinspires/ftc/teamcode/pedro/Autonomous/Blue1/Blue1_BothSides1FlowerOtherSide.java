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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Blue1_BothSides1FlowerOtherSide", group = "Blue1")
public class Blue1_BothSides1FlowerOtherSide extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(82.023, 132.0158, 270);
    private final Pose shoot1 = poseFactory.of(82.023, 123.421, 270);
    private final Pose collectPollenStart = poseFactory.of(82.023, 123.421, 0);
    private final Pose collectPollen = poseFactory.of(132.0802, 92.5798, 0);
    private final Pose collectPollenControl1 = poseFactory.of(101.8922, 99.3089, 0);
    private final Pose point3 = poseFactory.of(108.7931, 61.9655, 86.7582);
    private final Pose point3Control1 = poseFactory.of(109.717, 80.1983, 0);
    private final Pose shoot2Start = poseFactory.of(108.7931, 61.9655, 90);
    private final Pose shoot2 = poseFactory.of(83.3621, 28.5029, 90);
    private final Pose shoot2Control1 = poseFactory.of(118.3103, 26.944, 0);
    private final Pose parkStart = poseFactory.of(83.3621, 28.5029, 0);
    private final Pose park = poseFactory.of(130.7249, 42.9735, 0);
    private final Pose parkControl1 = poseFactory.of(109.2443, 27.1293, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot1()),
                follow(follower, collectPollen()),
                follow(follower, path3()),
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

    public Path collectPollen() {
        return curve(collectPollenStart, collectPollenControl1, collectPollen).linear(collectPollenStart, collectPollen);
    }

    public Path path3() {
        return curve(collectPollen, point3Control1, point3).reverseTangent();
    }

    public Path shoot2() {
        return curve(shoot2Start, shoot2Control1, shoot2).linear(shoot2Start, shoot2);
    }

    public Path park() {
        return curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
}
