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
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Blue1", group = "Blue1")
public class Blue1 extends LinearOpMode {

    private Follower follower;

    Pose currentPose;

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
    private final ElapsedTime matchTimer = new ElapsedTime();

    private boolean hive1_up = false;
    private boolean hive2_up = false;
    private boolean done_execute = false;
    private boolean loaded = false;
    private boolean test = false;
    private boolean forceParked = false;
    //Poses
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(83, 132.5, 270);
    private final Pose shoot1 = poseFactory.of(114, 123, 235);
    private final Pose blueGardenStart = poseFactory.of(119, 130, 0);
    private final Pose blueGardenEnd = poseFactory.of(130, 130, 0);
    private final Pose blueGardenEnd_shoot1CP = poseFactory.of(127.5,122.5, 0);
    //Shoot Other Side
    private final Pose shoot2 = poseFactory.of(114, 18, 125);
    private final Pose blueFlower1Start = poseFactory.of(94.5, 22.5, 270);
    private final Pose blueFlower1End = poseFactory.of(94.5, 15.5, 270);
    private final Pose home = poseFactory.of(131, 46, 180);
    private final Pose shoot2_homeCP = poseFactory.of(114, 44, 0);
    //Shoot Same Side
    private final Pose blueFlower2Start = poseFactory.of(118, 91.5, 0);
    private final Pose blueFlower2End = poseFactory.of(125.5, 91.5, 0);

    // Commands
    private void dontShoot() {
        leftCR.setPower(0);
        rightCR.setPower(0);
        integration.setPower(0);
    }

    public void lowerDolphin(){
        if(test == false) {
            leftDolphin.setPosition(0.535);
            rightDolphin.setPosition(0.55);
        }
    }
    public void liftDolphin(){
        leftDolphin.setPosition(0.1);
        rightDolphin.setPosition(0.1);
    }
    // --- Shooting method ---
    public void shoot() {
        leftCR.setPower(-1);
        rightCR.setPower(1);
        integration.setPower(-1);
    }
    public void doneExecute() {
        done_execute = true;
    }
    public void doneShooting(){loaded = false;}
    public void doneCollecting(){loaded = true;}
    // Autonomous Routines
    public Command Start() {
        return sequential(
                instant(this::liftDolphin),
                follow(follower, start_shoot1()),
                // shoot
                waitMs(1000),
                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
                instant(this::dontShoot),
                // collect Garden
                follow(follower, shoot1_blueGardenStart()),
                follow(follower, blueGardenStart_blueGardenEnd()),
                waitMs(1000),
                // get ready to shoot again
                follow(follower, blueGardenEnd_shoot1()),
                // check for hive tip
                waitMs(1000),
                instant(this::doneExecute)
        );
    }
    public Command ShootAtPosition(){
        //Just Shoot
        return sequential(
                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
                instant(this::dontShoot),
                instant(this::doneShooting),
                instant(this::doneExecute)
        );
    }
    public Command ShootPosition2() {
        return sequential(
                // go to other side and shoot
                follow(follower, goToShoot2()),
                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
                instant(this::dontShoot),
                instant(this::doneShooting),
                instant(this::doneExecute)
        );
    }
    public Command ShootPosition1() {
        return sequential(
                // shoot on same side to tip hive
                follow(follower, goToShoot1()),
                instant(this::shoot),
                waitMs(SHOOT_TIME_MS),
                instant(this::dontShoot),
                instant(this::doneShooting),
                instant(this::doneExecute)
        );
    }
    public Command collectFlower2(){
        return sequential(
                // shoot on same side to tip hive
                follow(follower, goToBlueFlower2Start()),
                instant(this::lowerDolphin),
                waitMs(LOWER_DOLPHIN_TIME_MS),
                follow(follower, blueFlower2Start_blueFlower2End()),
                waitMs(COLLECT_FLOWER_TIME_MS),
                follow(follower, blueFlower2End_blueFlower2Start()),
                instant(this::liftDolphin),
                // shoot on other side
                follow(follower, goToShoot1()),
                instant(this::doneCollecting),
                waitMs(1000),
                instant(this::doneExecute)

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
        currentPose = follower.pose();
        hive1_up = true;
        loaded = true;

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


        while (opModeInInit()){
            if (gamepad1.a){
                intake.setPower(-1);
            }else {
                intake.setPower(0);
            }
        }

        waitForStart();
        matchTimer.reset();
        resetRuntime();
        schedule(Start());

        while (opModeIsActive()) {
            llResult = limelight.getLatestResult();
            currentPose = follower.pose();
            //telemetry.addLine("Runtime: " + getRuntime());
            intake.setPower(-1);
            shooter.setVelocity(SHOOTER_VELOCITY);
            telemetry.addData("loaded = ", loaded);
            telemetry.addData("Done Execute = ", done_execute);
            telemetry.addData("Hive up = ", (hive1_up)?"1":((hive2_up)?"2":"Do not Know"));
            follower.update();
            Scheduler.execute();
            if (!forceParked && matchTimer.seconds() >= 25.0) {
                Scheduler.reset();
                schedule(follow(follower, goToHome()));
                forceParked = true;
                done_execute = false;
            }

            if (done_execute) {
                done_execute = false;
                if (loaded) {
                    telemetry.addLine("Loaded");
                    if (llResult != null && llResult.isValid() && !llResult.getFiducialResults().isEmpty()) {
                        int IdNum = llResult.getFiducialResults().get(0).getFiducialId();
                        if (IdNum == 42 || IdNum == 43 || IdNum == 44 || IdNum == 45)
                            hive1_up = true;
                        else if (IdNum == 37 || IdNum == 38 || IdNum == 39 || IdNum == 40)
                            hive2_up = true;
                        telemetry.addData("No Hive Tip, Id = ", IdNum);
                        Scheduler.schedule(ShootAtPosition());
                    } else {
                        //No Apriltags detected -> Yes hive Tip -> schedule(ifHiveTip)
                        if (hive1_up) {
                            telemetry.addLine("Hive Tip Identified");
                            telemetry.addLine("Hive 1 was Up, going to Hive 2");
                            Scheduler.schedule(ShootPosition2());
                        } else if (hive2_up) {
                            telemetry.addLine("Hive Tip Identified");
                            telemetry.addLine("Hive 2 was Up, going to Hive 1");
                            Scheduler.schedule(ShootPosition1());
                        } else {
                            telemetry.addLine("Hive Tip Identified");
                            telemetry.addLine("Not Sure Which Hive was Up, Just Shooting");
                            Scheduler.schedule(ShootAtPosition());
                        }
                    }
                }
                else {
                    telemetry.addLine("Not Loaded");
                    telemetry.addLine("Done Shooting, Going to Load from the Flower");
                    Scheduler.schedule(collectFlower2());
                }
            }
            telemetry.update();
//            telemetry.addData("x", follower.pose().x());
//            telemetry.addData("y", follower.pose().y());
//            telemetry.addData("heading", follower.pose().heading());
//
//            if (follower.currentPath() != null) {
//                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
//                telemetry.addData("Path number", follower.pathIndex());
//            }
//            telemetry.update();
        }
    }
    //Paths
    //StartRoutine
    public Path start_shoot1() {
        return Paths.line(start, shoot1).linear(start, shoot1);
    }
    public Path shoot1_blueGardenStart() {
        return Paths.line(shoot1, blueGardenStart).linear(0, 0);
    }

    public Path blueGardenStart_blueGardenEnd() {
        return Paths.line(blueGardenStart, blueGardenEnd).linear(blueGardenStart, blueGardenEnd);
    }

    public Path blueGardenEnd_shoot1() {
        return Paths.curve(blueGardenEnd, blueGardenEnd_shoot1CP, shoot1).linear(blueGardenEnd, shoot1);
    }
    //ShootPosition2
    public Path shoot1_shoot2() {
        return Paths.line(shoot1, shoot2).linear(shoot1, shoot2);
    }
    public Path goToShoot1(){
        return Paths.line(currentPose, shoot1).linear(currentPose, shoot1);
    }
    public Path goToShoot2(){
        return Paths.line(currentPose, shoot2).linear(currentPose, shoot2);
    }
    public Path goToBlueFlower2Start(){
        return Paths.line(currentPose, blueFlower2Start).linear(currentPose, blueFlower2Start);
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
    // ShootPosition1
    public Path shoot1_blueFlower2Start() {
        return Paths.line(shoot1, blueFlower2Start).linear(shoot1, blueFlower2Start);
    }

    public Path blueFlower2Start_blueFlower2End() {
        return Paths.line(blueFlower2Start, blueFlower2End).linear(blueFlower2Start, blueFlower2End);
    }

    public Path blueFlower2End_blueFlower2Start() {
        return Paths.line(blueFlower2End, blueFlower2Start).linear(blueFlower2End, blueFlower2Start);
    }
    public Path goToHome(){
        return Paths.line(currentPose, home).linear(currentPose, home);
    }

    public Path blueFlower2Start_shoot2() {
        return Paths.line(blueFlower2Start, shoot2).linear(blueFlower2Start, shoot2);
    }
}
