package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoTurn {
    private final Servo ServoTurn;

    public ServoTurn(HardwareMap hardwareMap){
        ServoTurn = hardwareMap.get(Servo.class,"ServoTurn");

    }

    public void setServoTurn(double position){
        ServoTurn.setPosition(position);
    }
}
