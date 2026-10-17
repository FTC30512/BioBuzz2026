package org.firstinspires.ftc.teamcode.pedro.Autonomous.Blue1;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Blue1_BothSidesNoFlowersShootOtherSide", group = "Blue1")
public class Blue1_BothSidesNoFlowersShootOtherSide extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(82.5, 132, 270);
    private final Pose shoot1 = poseFactory.of(84, 119, 270);
    private final Pose approachGardenStart = poseFactory.of(84, 119, 270);
    private final Pose approachGarden = poseFactory.of(120, 132, 0);
    private final Pose approachGardenControl1 = poseFactory.of(98.0061, 129.4182, 0);
    private final Pose collectPollenStart = poseFactory.of(120, 132, 0);
    private final Pose collectPollen = poseFactory.of(132, 132, 0);
    private final Pose shoot2Start = poseFactory.of(132, 132, 0);
    private final Pose shoot2 = poseFactory.of(112.3637, 73.9032, 90);
    private final Pose shoot2Control1 = poseFactory.of(112.8926, 118.7277, 0);
    private final Pose parkStart = poseFactory.of(112.3637, 73.9032, 90);
    private final Pose park = poseFactory.of(84, 21, 90);
    private final Pose parkControl1 = poseFactory.of(108.6728, 22.4148, 0);
    private final Pose point6Start = poseFactory.of(84, 21, 90);
    private final Pose point6 = poseFactory.of(130, 46, 180);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot1()),
                follow(follower, approachGarden()),
                follow(follower, collectPollen()),
                follow(follower, shoot2()),
                follow(follower, park()),
                follow(follower, path6())
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

    public Path approachGarden() {
        return Paths.curve(approachGardenStart, approachGardenControl1, approachGarden).linear(approachGardenStart, approachGarden);
    }

    public Path collectPollen() {
        return Paths.line(collectPollenStart, collectPollen).linear(collectPollenStart, collectPollen);
    }

    public Path shoot2() {
        return Paths.curve(shoot2Start, shoot2Control1, shoot2).linear(shoot2Start, shoot2);
    }

    public Path park() {
        return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
    }

    public Path path6() {
        return Paths.line(point6Start, point6).linear(point6Start, point6);
    }
}
