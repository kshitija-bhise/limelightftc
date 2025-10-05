package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


/*
* In this code, when the april tag is moved at a certain distance for example, here i have taken 3 positions 50,30,20 where
* each position is assigned a servo position without any moment of the robot.
*/

@Autonomous(name = "Servo_Limelight")
public class ServoTest extends LinearOpMode {
    private Limelight3A limelight;
    private Servo armservo;
    private DcMotorEx RF, LF, LR, RR;
    private DistanceEstimator distanceEstimator;

    private final double targetDistance = 30.0; // inches

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        armservo = hardwareMap.get(Servo.class, "Servo");

        RF = hardwareMap.get(DcMotorEx.class, "RF");
        RR = hardwareMap.get(DcMotorEx.class, "RR");
        LF = hardwareMap.get(DcMotorEx.class, "LF");
        LR = hardwareMap.get(DcMotorEx.class, "LR");

        armservo.setPosition(0.7);

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LR.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight.pipelineSwitch(6);
        limelight.start();

        telemetry.setMsTransmissionInterval(11);

        distanceEstimator = new DistanceEstimator(limelight, 32.0, 6.0, 27.0);

        waitForStart();

        while (opModeIsActive()){
            if(!distanceEstimator.hasTarget()){
                telemetry.addData("Status", "No Target");
                stop();
                telemetry.update();
            }

            double distance = distanceEstimator.getDistanceInches();
            double tx = distanceEstimator.getTx();

            telemetry.addData("Distance", distance);
            telemetry.addData("Tx", tx);

            double currentPos = armservo.getPosition();
            double targetPos = 0.0;
            double kP_servo = 0.02;


            if (distance >= 50) {
                targetPos = 0.9;

            } else if (distance >= 30) {
                targetPos = 0.6;
            }

            if (distance <= 20) {
                targetPos = 0.35;

            }

            double servo_error = targetPos - currentPos;
            double servoStep = currentPos + (kP_servo * servo_error);
            servoStep = Math.max(0.0, Math.min(1.0, servoStep));

            armservo.setPosition(servoStep);

            telemetry.addData("distance", distance);
            telemetry.update();
        }
    }

}

