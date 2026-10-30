package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import com.qualcomm.hardware.limelightvision.Limelight3A;


@TeleOp
public class LimelightTest extends LinearOpMode {
    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private GoBildaPinpointDriver pinpoint;
    Limelight3A limelight;
    private LLResult llResult;
    private boolean isTurning = false;
    private double actYaw = 0;
    private double maxTurnSpeed = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware ();
        waitForStart();
        while(opModeIsActive())
        {
            pinpoint.update ();
            double currentHeading = getHeading();
            limelight.updateRobotOrientation(getRawHeading());
            llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()){
                telemetry.addData("Tx", llResult.getTx());
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Ta", llResult.getTa());
                telemetry.addLine("Current Heading angle" + getHeading());
                telemetry.addLine("pinpoint angle" + pinpoint.getHeading(AngleUnit.DEGREES));

                telemetry.update();
            }
            // Start the turn when X is pressed and a target is visible
            if (gamepad1.x && llResult != null && llResult.isValid() && !isTurning) {
                double targetYaw = -(llResult.getTx() - 4.5);
                actYaw = currentHeading + targetYaw;
                isTurning = true;
            }

            if (isTurning) {
                double error = actYaw - currentHeading;

                if (Math.abs(error) > 0.5) {
                    double kp = 0.02;
                    double power = error * kp;

                    // Bound the power limits smoothly
                    if (power > 0) {
                        power = Math.min(Math.max(power, 0.15), maxTurnSpeed);
                    } else {
                        power = Math.min(Math.max(power, -maxTurnSpeed), -0.15);
                    }

                    telemetry.addData("Tx", llResult.getTx());
                    telemetry.addData("Ty", llResult.getTy());
                    telemetry.addData("Ta", llResult.getTa());
                    telemetry.addData("Current Heading: ", currentHeading);
                    telemetry.addData("Target Angle", actYaw);
                    telemetry.addData("Actual Power", power);

                    // Uncomment this to make the wheels physically move:
                    setDrivePower(-power, -power, power, power);
                } else {
                    // Target reached
                    setDrivePower(0, 0, 0, 0);
                    isTurning = false;
                }
            }

            telemetry.update();

        }

    }
    // --- Hardware initialization method ---
    private void initHardware() {
        // Motors
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        // Sensors
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets ( -7.0, -5.625, DistanceUnit.INCH );
        pinpoint.setEncoderDirections ( GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD );
        pinpoint.setEncoderResolution ( GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD );
        pinpoint.resetPosAndIMU ();

        for (DcMotor m : new DcMotor[]{leftFront, leftRear, rightFront, rightRear}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        limelight = hardwareMap.get ( Limelight3A.class, "limelight" );
        limelight.pipelineSwitch(1);
        limelight.start();
    }
    private double getHeading() {
        double angle = getRawHeading();
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    private double getRawHeading() {
        return pinpoint.getHeading ( AngleUnit.DEGREES );
    }

    public void pid_turn_by_gyro(double targetYaw, double speed){
        targetYaw=-targetYaw;
        double currentYaw = getHeading();
        double error;
        double actYaw = getHeading() + targetYaw;
        double kp = 0.02;
        while(Math.abs(actYaw - getHeading()) > 0.5){
            sleep(100);
            error = kp * (actYaw - getHeading());
            double power;
            if(error > 0)
                power = Math.min(Math.max(error, 0.15), speed);
            else
                power = Math.min(Math.max(error, -speed), -0.15);

            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addLine("Current Heading angle" + getHeading());
            telemetry.addLine( "Target Angle" + targetYaw);
            telemetry.addLine("Actual Target Yaw" + actYaw);
            telemetry.addLine("Actual Power " + power);
            telemetry.update();
            //setDrivePower(-power, -power, power, power );
        }
        setDrivePower(0,0,0,0 );

    }
    private void setDrivePower(double lfPower, double lrPower, double rfPower, double rrPower)
    {
        leftFront.setPower(lfPower);
        leftRear.setPower(lrPower);
        rightFront.setPower(rfPower);
        rightRear.setPower(rrPower);
    }

    public void resetHeading() {
        pinpoint.resetPosAndIMU ();
    }
}
