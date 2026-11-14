package org.firstinspires.ftc.teamcode.pedro.Autonomous.Blue1;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@Autonomous(name = "TestDetectionOnOldRobot", group = "Blue1")
public class TestDetectionOnOldRobot extends LinearOpMode {

    private Follower follower;

    //Limelight
    private IMU imu;
    private GoBildaPinpointDriver pinpoint;
    Limelight3A limelight;
    private LLResult llResult;
    // Motors & Servos
//    private DcMotor intake, integration;
//    private DcMotorEx shooter;
//    private CRServo leftCR, rightCR;
//    private Servo leftDolphin, rightDolphin;
    // Important Constants
    private static final double SHOOTER_VELOCITY = 1350;
    private final double Kp = 255.0, Ki = 0.0, Kd = 0.0, Kf = 11.62;
    private static final long SHOOT_TIME_MS = 2000;
    private static final long COLLECT_FLOWER_TIME_MS = 3000;
    private static final long LOWER_DOLPHIN_TIME_MS = 1000;
    private int shootNumber = 1;
    private boolean done_execute = false;
    //Poses
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(83, 132.5, 270);
    private final Pose homeShoot = poseFactory.of(114, 123, 235);
    private final Pose collectGardenStart = poseFactory.of(120, 131.5, 0);
    private final Pose collectGardenEnd = poseFactory.of(131, 131.5, 0);
    private final Pose approachHomeShoot = poseFactory.of(109, 121.5, 270);

    //Shoot Other Side
    private final Pose awayShoot = poseFactory.of(114, 18, 125);
    private final Pose collectAwayFlowerStart = poseFactory.of(94.5, 22.5, 270);
    private final Pose collectAwayFlowerEnd = poseFactory.of(94.5, 12.5, 270);
    private final Pose park = poseFactory.of(131, 46, 180);
    private final Pose awayShoot_parkCP = poseFactory.of(114, 44, 0);
    //Shoot Same Side
    private final Pose collectHomeFlowerStart = poseFactory.of(120, 94.5, 0);
    private final Pose collectHomeFlowerEnd = poseFactory.of(127, 94.5, 0);

    // Commands
//    private void dontShoot() {
//        leftCR.setPower(0);
//        rightCR.setPower(0);
//        integration.setPower(0);
//    }
//
//    public void lowerDolphin(){
//        leftDolphin.setPosition(0.535);
//        rightDolphin.setPosition(0.55);
//    }
//    public void liftDolphin(){
//        leftDolphin.setPosition(0.1);
//        rightDolphin.setPosition(0.1);
//    }
//    // --- Shooting method ---
//    public void shoot() {
//        leftCR.setPower(-1);
//        rightCR.setPower(1);
//        integration.setPower(1);
//    }
    public void doneExecute() {
        done_execute = true;}
    // Autonomous Routines
    public void shootPrint() {
        telemetry.addLine("shoot" + shootNumber);
        telemetry.update();
        shootNumber++;
    }
    public Command Start() {
        return sequential(
//                instant(this::liftDolphin),
                follow(follower, start_homeShoot()),

                //Shoot
//                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
//                instant(this::dontShoot),
                instant(this::shootPrint),
                //Collect Garden
                follow(follower, homeShoot_collectGardenStart()),
                follow(follower, collectGardenStart_collectGardenEnd()),
                //Get Ready to Shoot Again
                follow(follower, collectGardenEnd_approachHomeShoot()),
                follow(follower, approachHomeShoot_homeShoot()),
                //Check for hive tip
                instant(this::doneExecute),
                waitMs(1000)
        );
    }
    public Command ShootOtherSide() {
        return sequential(
                //go to other side and shoot
                follow(follower, homeShoot_awayShoot()),
//                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
//                instant(this::dontShoot),
                instant(this::shootPrint),
                // collect away flower
//                follow(follower, awayShoot_collectAwayFlowerStart()),
////                instant(this::lowerDolphin),
//                waitMs(LOWER_DOLPHIN_TIME_MS),
//                follow(follower, collectAwayFlowerStart_collectAwayFlowerEnd()),
//                waitMs(COLLECT_FLOWER_TIME_MS),
//                follow(follower, collectAwayFlowerEnd_collectAwayFlowerStart()),
////                instant(this::liftDolphin),
//                //shoot on other side for second tip
//                follow(follower, collectAwayFlowerStart_awayShoot()),
////                instant(this::shoot),
//                waitMs(SHOOT_TIME_MS),
////                instant(this::dontShoot),
//                instant(this::shootPrint),
                follow(follower, awayShoot_park())
        );
    }
    public Command ShootSameSide() {
        return sequential(
                // Shoot on same side to tip hive
//                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
//                instant(this::dontShoot),
                instant(this::shootPrint),
                // collect home flower
                follow(follower, homeShoot_collectHomeFlowerStart()),
//                instant(this::lowerDolphin),
                waitMs(LOWER_DOLPHIN_TIME_MS),
                follow(follower, collectHomeFlowerStart_collectHomeFlowerEnd()),
                waitMs(COLLECT_FLOWER_TIME_MS),
                follow(follower, collectHomeFlowerEnd_collectHomeFlowerStart()),
//                instant(this::liftDolphin),
                // shoot on other side
                follow(follower, collectHomeFlowerStart_awayShoot()),
//                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
//                instant(this::dontShoot),
                instant(this::shootPrint),
                // collect away flower
//                follow(follower, awayShoot_collectAwayFlowerStart()),
////                instant(this::lowerDolphin),
//                waitMs(LOWER_DOLPHIN_TIME_MS),
//                follow(follower, collectAwayFlowerStart_collectAwayFlowerEnd()),
//                waitMs(COLLECT_FLOWER_TIME_MS),
//                follow(follower, collectAwayFlowerEnd_collectAwayFlowerStart()),
////                instant(this::liftDolphin),
//                // awayShoot again
//                follow(follower, collectAwayFlowerStart_awayShoot()),
////                instant(this::shoot),
//                waitMs(SHOOT_TIME_MS),
////                instant(this::dontShoot),
//                instant(this::shootPrint),
                // park
                follow(follower, awayShoot_park())
        );
    }
    public Command ifHiveTip() {
        return sequential(
                follow(follower, homeShoot_awayShoot())
        );
    }
    public Command ifNoHiveTip() {
        return sequential(
                follow(follower,homeShoot_collectGardenStart())
        );
    }
    @Override
    public void runOpMode() {
        Scheduler.reset();

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        limelight.start();



        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

//        leftCR = hardwareMap.get(CRServo.class, "leftCR");
//        rightCR = hardwareMap.get(CRServo.class, "rightCR");
//        leftDolphin = hardwareMap.get(Servo.class, "leftDolphin");
//        rightDolphin = hardwareMap.get(Servo.class, "rightDolphin");
//        leftDolphin.setDirection(Servo.Direction.REVERSE);
//        intake = hardwareMap.get(DcMotor.class, "Intake");
//        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
//        shooter.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf);
//        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
//        integration = hardwareMap.get(DcMotor.class, "Integration");

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        resetRuntime();
        schedule(Start());

        while (opModeIsActive()) {
            llResult = limelight.getLatestResult();
            telemetry.addLine("Runtime: " + getRuntime());
//            intake.setPower(-1);
//            shooter.setVelocity(SHOOTER_VELOCITY);

            follower.update();
            Scheduler.execute();
            if (done_execute) {
                done_execute = false;
                if (llResult != null && llResult.isValid()) {
                    if (!llResult.getFiducialResults().isEmpty()) {
                        //Apriltags detected -> No Hive Tip -> schedule(ifNoHiveTip)
                        telemetry.addLine("No Hive Tip");
                        telemetry.update();
                        Scheduler.schedule(ShootSameSide());
                    }
                } else {
                    //No Apriltags detected -> Yes hive Tip -> schedule(ifHiveTip)
                    telemetry.addLine("Hive Tip Identified");
                    telemetry.update();
                    Scheduler.schedule(ShootOtherSide());
                }
            }

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
    //Paths
    //StartRoutine
    public Path start_homeShoot() {
        return Paths.line(start, homeShoot).linear(start, homeShoot);
    }

    public Path homeShoot_collectGardenStart() {
        return Paths.line(homeShoot, collectGardenStart).linear(0, 0);
    }

    public Path collectGardenStart_collectGardenEnd() {
        return Paths.line(collectGardenStart, collectGardenEnd).linear(collectGardenStart, collectGardenEnd);
    }

    public Path collectGardenEnd_approachHomeShoot() {
        return Paths.line(collectGardenEnd, approachHomeShoot).linear(collectGardenEnd, approachHomeShoot);
    }

    public Path approachHomeShoot_homeShoot() {
        return Paths.line(approachHomeShoot, homeShoot).linear(approachHomeShoot, homeShoot);
    }
    //ShootOtherSide
    public Path homeShoot_awayShoot() {
        return Paths.line(homeShoot, awayShoot).linear(homeShoot, awayShoot);
    }

    public Path awayShoot_collectAwayFlowerStart() {
        return Paths.line(awayShoot, collectAwayFlowerStart).linear(270, 270);
    }

    public Path collectAwayFlowerStart_collectAwayFlowerEnd() {
        return Paths.line(collectAwayFlowerStart, collectAwayFlowerEnd).linear(collectAwayFlowerStart, collectAwayFlowerEnd);
    }

    public Path collectAwayFlowerEnd_collectAwayFlowerStart() {
        return Paths.line(collectAwayFlowerEnd, collectAwayFlowerStart).linear(collectAwayFlowerEnd, collectAwayFlowerStart);
    }

    public Path collectAwayFlowerStart_awayShoot() {
        return Paths.line(collectAwayFlowerStart, awayShoot).linear(135, 135);
    }

    public Path awayShoot_park() {
        return Paths.curve(awayShoot, awayShoot_parkCP, park).linear(awayShoot, park);
    }
    // ShootSameSide
    public Path homeShoot_collectHomeFlowerStart() {
        return Paths.line(homeShoot, collectHomeFlowerStart).linear(homeShoot, collectHomeFlowerStart);
    }

    public Path collectHomeFlowerStart_collectHomeFlowerEnd() {
        return Paths.line(collectHomeFlowerStart, collectHomeFlowerEnd).linear(collectHomeFlowerStart, collectHomeFlowerEnd);
    }

    public Path collectHomeFlowerEnd_collectHomeFlowerStart() {
        return Paths.line(collectHomeFlowerEnd, collectHomeFlowerStart).linear(collectHomeFlowerEnd, collectHomeFlowerStart);
    }

    public Path collectHomeFlowerStart_awayShoot() {
        return Paths.line(collectHomeFlowerStart, awayShoot).linear(collectHomeFlowerStart, awayShoot);
    }
}
