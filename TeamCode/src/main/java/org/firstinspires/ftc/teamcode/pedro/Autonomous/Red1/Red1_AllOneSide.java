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

    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorY(70.75);

    private final Pose start = poseFactory.of(59, 133, -90);
    private final Pose shoot1 = poseFactory.of(57.5, 123, -90);
    private final Pose collectGardenPart1Start = poseFactory.of(57.5, 123, -90);
    private final Pose collectGardenPart1 = poseFactory.of(21.5, 132, 180);
    private final Pose collectGardenPart1Control1 = poseFactory.of(43.5, 124, 0);
    private final Pose collectGardenPart2Start = poseFactory.of(21.5, 132, 180);
    private final Pose collectGardenPart2 = poseFactory.of(9.5, 132, 180);
    private final Pose shoot2Start = poseFactory.of(9.5, 132, 180);
    private final Pose shoot2 = poseFactory.of(57.5, 123, -90);
    private final Pose collectHomeFlowerStart = poseFactory.of(57.5, 123, -90);
    private final Pose collectHomeFlower = poseFactory.of(14.5, 94, 180);
    private final Pose shoot3Start = poseFactory.of(14.5, 94, 180);
    private final Pose shoot3 = poseFactory.of(57.5, 123, -90);
    private final Pose parkStart = poseFactory.of(57.5, 123, -90);
    private final Pose park = poseFactory.of(11.5, 46, 0);
    private final Pose parkControl1 = poseFactory.of(11.5, 100, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot1()),
                follow(follower, collectGardenPart1()),
                follow(follower, collectGardenPart2()),
                follow(follower, shoot2()),
                follow(follower, collectHomeFlower()),
                follow(follower, shoot3()),
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

    public Path collectGardenPart1() {
        return Paths.curve(collectGardenPart1Start, collectGardenPart1Control1, collectGardenPart1).linear(collectGardenPart1Start, collectGardenPart1);
    }

    public Path collectGardenPart2() {
        return Paths.line(collectGardenPart2Start, collectGardenPart2).linear(collectGardenPart2Start, collectGardenPart2);
    }

    public Path shoot2() {
        return Paths.line(shoot2Start, shoot2).linear(shoot2Start, shoot2);
    }

    public Path collectHomeFlower() {
        return Paths.line(collectHomeFlowerStart, collectHomeFlower).linear(collectHomeFlowerStart, collectHomeFlower);
    }

    public Path shoot3() {
        return Paths.line(shoot3Start, shoot3).linear(shoot3Start, shoot3);
    }

    public Path park() {
        return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
}
