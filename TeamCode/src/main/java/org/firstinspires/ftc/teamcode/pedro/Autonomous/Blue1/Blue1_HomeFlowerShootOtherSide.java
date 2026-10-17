package org.firstinspires.ftc.teamcode.pedro.Autonomous.Blue1;

import com.pedropathing.api.Paths;

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
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Blue1_HomeFlowerShootOtherSide", group = "Blue1")
public class Blue1_HomeFlowerShootOtherSide extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(82.5, 132, 270);
    private final Pose shoot1 = poseFactory.of(84, 123, 270);
    private final Pose collectPollenStart = poseFactory.of(84, 123, 270);
    private final Pose collectPollen = poseFactory.of(125.1684, 94.1158, 0);
    private final Pose collectPollenControl1 = poseFactory.of(101.8922, 99.3089, 0);
    private final Pose shoot2part1Start = poseFactory.of(125.1684, 94.1158, 0);
    private final Pose shoot2part1 = poseFactory.of(112.6088, 61.9655, 90);
    private final Pose shoot2part1Control1 = poseFactory.of(112.5788, 82.1061, 0);
    private final Pose shoot2part2Start = poseFactory.of(112.6088, 61.9655, 90);
    private final Pose shoot2part2 = poseFactory.of(84, 21, 90);
    private final Pose shoot2part2Control1 = poseFactory.of(110.0429, 30.7597, 0);
    private final Pose parkStart = poseFactory.of(84, 21, 90);
    private final Pose park = poseFactory.of(130.9169, 45.8534, 180);
    private final Pose parkControl1 = poseFactory.of(109.2443, 27.1293, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot1()),
                follow(follower, collectPollen()),
                follow(follower, shoot2part1()),
                follow(follower, shoot2part2()),
                follow(follower, park())
        );
    }

    @Override
    public void runOpMode() {
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            shooter.setPower(-1);
            intake.setPower(1);

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
        return Paths.line(start, shoot1).linear(start, shoot1);
    }

    public Path collectPollen() {
        return Paths.curve(collectPollenStart, collectPollenControl1, collectPollen).linear(collectPollenStart, collectPollen);
    }

    public Path shoot2part1() {
        return Paths.curve(shoot2part1Start, shoot2part1Control1, shoot2part1).linear(shoot2part1Start, shoot2part1);
    }

    public Path shoot2part2() {
        return Paths.curve(shoot2part2Start, shoot2part2Control1, shoot2part2).linear(shoot2part2Start, shoot2part2);
    }

    public Path park() {
        return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
}
