// Controls:
// gamepad1.b = resetHeading (software offset, safe mid-match)
// gamepad1.y = reverse intake
// gamepad1.dpad_up = turn off intake
// gamepad1.right_trigger = auto-aim then shoot
// gamepad1.left_bumper = lower/lift dolphin flap (toggle)

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Blue TeleOp", group = "Main")
public class BlueTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    // --- Motors & Servos ---
    private DcMotor leftFront, leftRear, rightFront, rightRear, intake, integration;
    private DcMotorEx shooter;
    private CRServo leftCR, rightCR;
    private Servo leftDolphin, rightDolphin;

    // --- Sensors ---
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    private LLResult llResult;

    // --- Helper Classes ---
    private Movement movement;

    // --- Heading Offset ---
    private double headingOffsetDeg = 0.0;

    // --- Shooter & Dolphin Variables ---
    private double Kp = 255.0, Ki = 0.0, Kd = 0.0, Kf = 11.62;
    private boolean is_lowered = false;
    private double tag_offset = 0;
    private boolean lastLeftBumper = false;

    // --- Auto-Aim & Shoot State Machine ---
    private enum ShootState {
        IDLE,
        AIMING,
        SHOOTING
    }

    private ShootState shootState = ShootState.IDLE;
    private double targetHeadingDeg = 0;
    private double lastTurnError = 0;
    private ElapsedTime aimTimer = new ElapsedTime();
    private ElapsedTime settleTimer = new ElapsedTime();

    // --- Turning PD Constants ---
    private static final double TURN_KP = 0.028;        // Snap speed toward target
    private static final double TURN_KD = 0.0025;       // Damping to prevent overshoot/oscillation
    private static final double MIN_TURN_POWER = 0.1;  // Minimum power to overcome static friction
    private static final double MAX_TURN_POWER = 0.65;  // Maximum turn speed
    private static final double AIM_TOLERANCE_DEG = 2;

    @Override
    public void runOpMode() {
        // --- Initialize hardware ---
        initHardware();
        liftDolphin();

        // --- Initialize helpers ---
        movement = new Movement(leftFront, leftRear, rightFront, rightRear, gamepad1, telemetry);

        telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
        telemetry.addLine("Initialized. Press PLAY to start.");
        telemetry.update();
        gamepad1.rumble(500);
        double ta;
        double shooter_speed=1350;

        waitForStart();

        // --- Main loop ---
        while (opModeIsActive()) {
            shooter_speed = 1350;


            shooter.setVelocity(shooter_speed);
            pinpoint.update();

            if (gamepad1.b) {
                resetHeading();
            }

            // --- Intake ---
            if (gamepad1.y) {
                intake.setPower(-1);
            } else {
                intake.setPower(1);
            }

            if (gamepad1.dpad_up) {
                intake.setPower(0);
            }

            // --- Limelight Update ---
            limelight.updateRobotOrientation(getRawHeading());
            llResult = limelight.getLatestResult();

            // --- Driving & Auto-Aim/Shoot ---
            boolean triggerPressed = gamepad1.right_trigger > 0.5;

            if (triggerPressed) {
                handleAutoAimAndShoot();
            } else {
                shootState = ShootState.IDLE;
                leftCR.setPower(0);
                rightCR.setPower(0);
                integration.setPower(0);

                // Only allow manual driving when not auto-aiming/shooting
                movement.drive(Math.toRadians(getHeading()));
            }

            // --- Dolphin Flap Toggle ---
            boolean currentLeftBumper = gamepad1.left_bumper;
            if (currentLeftBumper && !lastLeftBumper) {
                if (is_lowered) {
                    liftDolphin();
                } else {
                    lowerDolphin();
                }
            }
            lastLeftBumper = currentLeftBumper;

            // --- Telemetry ---
            telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
            telemetry.addData("Shoot State", shootState);
            telemetry.addData("Current Heading", String.format("%.2f", getHeading()));
            telemetry.addData("Target Heading", String.format("%.2f", targetHeadingDeg));
            if (llResult != null && llResult.isValid()) {
                telemetry.addData("Tag Detected", true);
                telemetry.addData("Tx", String.format("%.2f", llResult.getTx()));
                telemetry.addData("Ty", String.format("%.2f", llResult.getTy()));
                telemetry.addData("Ta", String.format("%.2f", llResult.getTa()));
            } else {
                telemetry.addData("Tag Detected", false);
            }
            telemetry.update();
        }
    }

    // --- Hardware initialization method ---
    private void initHardware() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        intake = hardwareMap.dcMotor.get("Intake");
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");

        integration = hardwareMap.dcMotor.get("Integration");
        leftCR = hardwareMap.get(CRServo.class, "leftCR");
        rightCR = hardwareMap.get(CRServo.class, "rightCR");

        leftDolphin = hardwareMap.get(Servo.class, "leftDolphin");
        rightDolphin = hardwareMap.get(Servo.class, "rightDolphin");
        leftDolphin.setDirection(Servo.Direction.REVERSE);

        shooter.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{leftFront, leftRear, rightFront, rightRear}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Pinpoint Odometry
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-7.0, -5.625, DistanceUnit.INCH);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.resetPosAndIMU();

        ElapsedTime calibTimer = new ElapsedTime();
        while (!isStopRequested() && calibTimer.seconds() < 1.0) {
            pinpoint.update();
            if (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
                break;
            }
            sleep(20);
        }

        headingOffsetDeg = pinpoint.getHeading(AngleUnit.DEGREES);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();

        runtime.reset();
    }

    public void lowerDolphin() {
        leftDolphin.setPosition(0.535);
        rightDolphin.setPosition(0.55);
        is_lowered = true;
    }

    public void liftDolphin() {
        leftDolphin.setPosition(0.1);
        rightDolphin.setPosition(0.1);
        is_lowered = false;
    }

    // --- Aim First, Then Shoot ---
    private void handleAutoAimAndShoot() {
        switch (shootState) {
            case IDLE:
                if (llResult != null && llResult.isValid()) {
                    // Snapshot target heading using Pinpoint + Limelight Tx
                    // NOTE: If robot turns away from the tag, change '-' to '+' below
                    targetHeadingDeg = normalizeAngle(getHeading() - (llResult.getTx() + tag_offset));
                    lastTurnError = getHeadingError(targetHeadingDeg);
                    aimTimer.reset();
                    settleTimer.reset();
                    shootState = ShootState.AIMING;
                } else {
                    // No AprilTag visible: stop wheels and fire immediately
                    setDrivePower(0, 0, 0, 0);
                    shootState = ShootState.SHOOTING;
                }
                break;

            case AIMING:
                double error = getHeadingError(targetHeadingDeg);
                double derivative = error - lastTurnError;
                lastTurnError = error;

                if (Math.abs(error) <= AIM_TOLERANCE_DEG) {
                    setDrivePower(0, 0, 0, 0);
                    // Wait 80ms settled on target before feeding shooter
                    if (settleTimer.milliseconds() > 80) {
                        shootState = ShootState.SHOOTING;
                    }
                } else {
                    settleTimer.reset();
                    double power = (error * TURN_KP) + (derivative * TURN_KD);

                    if (power > 0) {
                        power = Math.min(Math.max(power, MIN_TURN_POWER), MAX_TURN_POWER);
                    } else {
                        power = Math.max(Math.min(power, -MIN_TURN_POWER), -MAX_TURN_POWER);
                    }

                    setDrivePower(-power, -power, power, power);
                }

                // Safety timeout (1.2s) so robot always shoots even if blocked
                if (aimTimer.seconds() > 1.2) {
                    setDrivePower(0, 0, 0, 0);
                    shootState = ShootState.SHOOTING;
                }
                break;

            case SHOOTING:
                setDrivePower(0, 0, 0, 0);
                leftCR.setPower(-1);
                rightCR.setPower(1);
                integration.setPower(1);
                break;
        }
    }

    private double getHeadingError(double targetDeg) {
        return normalizeAngle(targetDeg - getHeading());
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private double getHeading() {
        return normalizeAngle(pinpoint.getHeading(AngleUnit.DEGREES) - headingOffsetDeg);
    }

    private double getRawHeading() {
        return normalizeAngle(pinpoint.getHeading(AngleUnit.DEGREES));
    }

    private void setDrivePower(double lfPower, double lrPower, double rfPower, double rrPower) {
        leftFront.setPower(lfPower);
        leftRear.setPower(lrPower);
        rightFront.setPower(rfPower);
        rightRear.setPower(rrPower);
    }

    public void resetHeading() {
        headingOffsetDeg = pinpoint.getHeading(AngleUnit.DEGREES);
    }
}