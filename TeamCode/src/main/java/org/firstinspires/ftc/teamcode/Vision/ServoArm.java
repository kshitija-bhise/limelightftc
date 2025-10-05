package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "ServoTest  ")
public class ServoArm extends LinearOpMode {
    private Limelight3A limelight;
    private Servo armservo;
    private DcMotorEx RF, LF, LR, RR;
    private DistanceEstimator distanceEstimator;
    private final double kP_forward = 0.05;

    private final double servoMin = 0.4;  // closest
    private final double servoMid = 0.5;  // at 30 inches
    private final double servoMax = 0.7;  // farthest
    private final double targetDistance = 30.0; // inches
    private final double maxDistance = 50.0;
    private final double dist_tolerance = 1.0;

    private final double kP = 0.8;
    private final double kI = 0.0;
    private final double kD = 0.1;

    private double integral = 0;
    private double lastError = 0;
    private double currentPos = servoMid;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        armservo = hardwareMap.get(Servo.class, "Servo");

        RF = hardwareMap.get(DcMotorEx.class, "RF");
        RR = hardwareMap.get(DcMotorEx.class, "RR");
        LF = hardwareMap.get(DcMotorEx.class, "LF");
        LR = hardwareMap.get(DcMotorEx.class, "LR");

        armservo.setPosition(0.5);

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LR.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight.pipelineSwitch(6);
        limelight.start();

        telemetry.setMsTransmissionInterval(11);

        distanceEstimator = new DistanceEstimator(limelight, 32.0, 6.0, 27.0);

        waitForStart();

        while (opModeIsActive()){
            LLResult result = limelight.getLatestResult();

            if (!distanceEstimator.hasTarget()) {
                telemetry.addData("Status", "No Target");
                stop();
                telemetry.update();
            }

            double distance = distanceEstimator.getDistanceInches();
            double tx = distanceEstimator.getTx();
            double targetServoPos;

            telemetry.addData("Distance", distance);
            telemetry.addData("Tx", tx);

            double error = distance - targetDistance;

            // Check if within tolerance
            if (Math.abs(error) <= dist_tolerance) {
                stop();
                telemetry.addLine("Target distance reached");
            } else {
                double power = kP_forward * error;

                power = Math.max(-0.5, Math.min(0.5, power));

                forward(power);
            }


            if(distance < targetDistance){
                double ratio = distance / targetDistance;
                targetServoPos = servoMin + (servoMid - servoMin) * ratio;
            }else{
                double ratio = (distance - targetDistance) / (maxDistance - targetDistance);
                targetServoPos = servoMin + (servoMax - servoMid) * (Math.min(ratio,1.0));

                double errorServo = targetServoPos - currentPos;
                integral += errorServo;
                double derivative = errorServo - lastError;
                lastError = errorServo;

                double pidOutput = (kP * error)*(kI * integral)*(kD * derivative);

                currentPos +=  pidOutput;

                currentPos = Math.max(servoMin, Math.min(servoMax, currentPos));

                armservo.setPosition(currentPos);

                telemetry.addData("distance", distance);
                telemetry.addData("Target Servo Pos", targetServoPos);
                telemetry.addData("Current Servo Pos", currentPos);
                telemetry.update();
            }
        }
    }

    public void  forward(double power){
        RF.setPower(power);
        RR.setPower(power);
        LF.setPower(power);
        LR.setPower(power);
    }
}
