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
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Blue1_AllOneSide", group = "Blue1")
public class Blue1_AllOneSide extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(85, 132, 270);
    private final Pose shoot1 = poseFactory.of(85, 123, 270);
    private final Pose collectGardenPart1 = poseFactory.of(119.3584, 131, 0);
    private final Pose collectGardenPart1Control1 = poseFactory.of(98.2245, 124.082, 0);
    private final Pose collectGardenPart2 = poseFactory.of(131, 131, 0);
    private final Pose shoot2 = poseFactory.of(85, 123, 270);
    private final Pose shoot2Control1 = poseFactory.of(97.8903, 125.0242, 0);
    private final Pose collectFlower = poseFactory.of(128, 94.5, 0);
    private final Pose shoot3 = poseFactory.of(85, 123, 270);
    private final Pose park = poseFactory.of(132.2288, 44.9007, 180);
    private final Pose parkControl1 = poseFactory.of(132.4007, 93.0769, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot1()),
                follow(follower, collectGardenPart1()),
                follow(follower, collectGardenPart2()),
                follow(follower, shoot2()),
                follow(follower, collectFlower()),
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
        return line(start, shoot1).linear(start, shoot1);
    }

    public Path collectGardenPart1() {
        return curve(shoot1, collectGardenPart1Control1, collectGardenPart1).linear(shoot1, collectGardenPart1);
    }

    public Path collectGardenPart2() {
        return line(collectGardenPart1, collectGardenPart2).linear(collectGardenPart1, collectGardenPart2);
    }

    public Path shoot2() {
        return curve(collectGardenPart2, shoot2Control1, shoot2).linear(collectGardenPart2, shoot2);
    }

    public Path collectFlower() {
        return line(shoot2, collectFlower).linear(shoot2, collectFlower);
    }

    public Path shoot3() {
        return line(collectFlower, shoot3).linear(collectFlower, shoot3);
    }

    public Path park() {
        return curve(shoot3, parkControl1, park).linear(shoot3, park);
    }
}
