package org.firstinspires.ftc.teamcode.pedro.Autonomous.Red1;

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

@Autonomous(name = "Red1_AllOneSide", group = "Red1")
public class Red1_AllOneSide extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59, 9, 90);
    private final Pose startshootpos1 = poseFactory.of(59, 24, 90);
    private final Pose shootpos1collectgarden1Start = poseFactory.of(59, 24, 90);
    private final Pose shootpos1collectgarden1 = poseFactory.of(21, 9, 180);
    private final Pose shootpos1collectgarden1Control1 = poseFactory.of(57.5692, 17.2571, 0);
    private final Pose collectgarden1collectgarden2Start = poseFactory.of(21, 9, 180);
    private final Pose collectgarden1collectgarden2 = poseFactory.of(12, 9, 180);
    private final Pose collectgarden2shootpos1Start = poseFactory.of(12, 9, 180);
    private final Pose collectgarden2shootpos1 = poseFactory.of(59, 24, 90);
    private final Pose collectgarden2shootpos1Control1 = poseFactory.of(36.7524, 20.0658, 0);
    private final Pose shootpos1collectflower1Start = poseFactory.of(59, 24, 90);
    private final Pose shootpos1collectflower1 = poseFactory.of(19, 47, 180);
    private final Pose shootpos1collectflower1Control1 = poseFactory.of(59, 24, 0);
    private final Pose collectflower1collectflower2 = poseFactory.of(14, 47.5, 180);
    private final Pose collectflower2shootpos1Start = poseFactory.of(14, 47.5, 180);
    private final Pose collectflower2shootpos1 = poseFactory.of(59, 24, 90);
    private final Pose point8Start = poseFactory.of(59, 24, 90);
    private final Pose point8 = poseFactory.of(11, 96, 0);
    private final Pose point8Control1 = poseFactory.of(13.4921, 47.682, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, startshootpos1()),
                follow(follower, shootpos1collectgarden1()),
                follow(follower, collectgarden1collectgarden2()),
                follow(follower, collectgarden2shootpos1()),
                follow(follower, shootpos1collectflower1()),
                follow(follower, collectflower1collectflower2()),
                follow(follower, collectflower2shootpos1()),
                follow(follower, path8())
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

    public Path startshootpos1() {
        return Paths.line(start, startshootpos1).linear(start, startshootpos1);
    }

    public Path shootpos1collectgarden1() {
        return Paths.curve(shootpos1collectgarden1Start, shootpos1collectgarden1Control1, shootpos1collectgarden1).linear(shootpos1collectgarden1Start, shootpos1collectgarden1);
    }

    public Path collectgarden1collectgarden2() {
        return Paths.line(collectgarden1collectgarden2Start, collectgarden1collectgarden2).linear(collectgarden1collectgarden2Start, collectgarden1collectgarden2);
    }

    public Path collectgarden2shootpos1() {
        return Paths.curve(collectgarden2shootpos1Start, collectgarden2shootpos1Control1, collectgarden2shootpos1).linear(collectgarden2shootpos1Start, collectgarden2shootpos1);
    }

    public Path shootpos1collectflower1() {
        return Paths.curve(shootpos1collectflower1Start, shootpos1collectflower1Control1, shootpos1collectflower1).linear(shootpos1collectflower1Start, shootpos1collectflower1);
    }

    public Path collectflower1collectflower2() {
        return Paths.line(shootpos1collectflower1, collectflower1collectflower2).constant(collectflower1collectflower2);
    }

    public Path collectflower2shootpos1() {
        return Paths.line(collectflower2shootpos1Start, collectflower2shootpos1).linear(collectflower2shootpos1Start, collectflower2shootpos1);
    }

    public Path path8() {
        return Paths.curve(point8Start, point8Control1, point8).linear(point8Start, point8);
    }
}
