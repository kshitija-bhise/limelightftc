package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.ServoAngle;

public class ServoAlign extends OpMode {

    private ServoAngle servoAngle;
    private DistanceEstimator currentDistance;
    private Servo shooterAligner;


    @Override
    public void init() {
        ServoAdjust servoControl = new ServoAdjust(shooterAligner,90,10,0.2,0.9);
    }

    public void loop() {

    }
}
