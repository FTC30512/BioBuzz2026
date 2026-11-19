package org.firstinspires.ftc.teamcode.pedro.Autonomous.Blue2;

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
@Autonomous(name = "Blue2", group = "Blue2")
public class Blue2 extends LinearOpMode {

    private Follower follower;

    //Limelight
    private IMU imu;
    private GoBildaPinpointDriver pinpoint;
    Limelight3A limelight;
    private LLResult llResult;
    // Motors & Servos
    private DcMotor intake, integration;
    private DcMotorEx shooter;
    private CRServo leftCR, rightCR;
    private Servo leftDolphin, rightDolphin;
    // Important Constants
    private static final double SHOOTER_VELOCITY = 1350;
    private final double Kp = 255.0, Ki = 0.0, Kd = 0.0, Kf = 11.62;
    private static final long SHOOT_TIME_MS = 2000;
    private static final long COLLECT_FLOWER_TIME_MS = 3000;
    private static final long LOWER_DOLPHIN_TIME_MS = 1000;
    private boolean hiveTipped = false;
    //Poses
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(81, 9, 90);
    private final Pose shoot2 = poseFactory.of(114, 18, 125);
    private final Pose start_shoot2CP = poseFactory.of(78.889, 24.6596, 0);
    private final Pose blueFlower1Start = poseFactory.of(94.5, 22.5, 270);
    private final Pose blueFlower1End = poseFactory.of(94.5, 15.5, 270);
    private final Pose home = poseFactory.of(131, 46, 180);
    private final Pose shoot2_homeCP = poseFactory.of(114, 44, 0);

    // Commands
    private void dontShoot() {
        leftCR.setPower(0);
        rightCR.setPower(0);
        integration.setPower(0);
    }

    public void lowerDolphin() {
        leftDolphin.setPosition(0.535);
        rightDolphin.setPosition(0.55);
    }

    public void liftDolphin() {
        leftDolphin.setPosition(0.1);
        rightDolphin.setPosition(0.1);
    }

    // --- Shooting method ---
    public void shoot() {
        leftCR.setPower(-1);
        rightCR.setPower(1);
        integration.setPower(1);
    }
    public void waitForTip() {
        hiveTipped = false;
        while (!hiveTipped) {
            if (llResult != null && llResult.isValid()) {
                if (!llResult.getFiducialResults().isEmpty()) {
                    hiveTipped = true;
                }
            }
        }
    }
    // Autonomous Routines
    public Command autoRoutine() {
        return sequential(
                //prepare to shoot
                follow(follower, start_shoot2()),
                //wait for hive to tip
                instant(this::waitForTip),
                //shoot
                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
                instant(this::dontShoot),
                //collect blueFlower1
                follow(follower, shoot2_blueFlower1Start()),
                instant(this::lowerDolphin),
                follow(follower, blueFlower1Start_blueFlower1End()),
                waitMs(LOWER_DOLPHIN_TIME_MS),
                follow(follower, blueFlower1End_blueFlower1Start()),
                instant(this::liftDolphin),
                //return to shoot position
                follow(follower, blueFlower1Start_shoot2()),
                //shoot
                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
                instant(this::dontShoot),
                // do waitForTip() again to make sure opponent didn't alerady
                follow(follower, shoot2_home())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        leftCR = hardwareMap.get(CRServo.class, "leftCR");
        rightCR = hardwareMap.get(CRServo.class, "rightCR");
        leftDolphin = hardwareMap.get(Servo.class, "leftDolphin");
        rightDolphin = hardwareMap.get(Servo.class, "rightDolphin");
        leftDolphin.setDirection(Servo.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "Intake");
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooter.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        integration = hardwareMap.get(DcMotor.class, "Integration");

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        resetRuntime();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            llResult = limelight.getLatestResult();
            telemetry.addLine("Runtime: " + getRuntime());
            intake.setPower(-1);
            shooter.setVelocity(SHOOTER_VELOCITY);

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

    //Paths
    public Path start_shoot2() {
        return Paths.curve(start, start_shoot2CP, shoot2).linear(start, shoot2);
    }

    public Path shoot2_blueFlower1Start() {
        return Paths.line(shoot2, blueFlower1Start).linear(shoot2, blueFlower1Start);
    }
    public Path blueFlower1Start_blueFlower1End() {
        return Paths.line(blueFlower1Start, blueFlower1End).linear(blueFlower1Start, blueFlower1End);
    }
    public Path blueFlower1End_blueFlower1Start() {
        return Paths.line(blueFlower1End, blueFlower1Start).linear(blueFlower1End, blueFlower1Start);
    }
    public Path blueFlower1Start_shoot2() {
        return Paths.line(blueFlower1Start, shoot2).linear(blueFlower1Start, shoot2);
    }
    public Path shoot2_home() {
        return Paths.curve(shoot2, shoot2_homeCP, home).linear(shoot2, home);
    }
}
