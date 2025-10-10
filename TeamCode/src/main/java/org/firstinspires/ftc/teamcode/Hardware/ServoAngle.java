package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoAngle {
    private final Servo ServoAdjust1;
    private final Servo ServoAdjust2;

    public ServoAngle(HardwareMap hardwareMap) {
        ServoAdjust1 = hardwareMap.get(Servo.class,"ServoAdjust1");
        ServoAdjust2 = hardwareMap.get(Servo.class,"ServoAdjust2");

//        ServoAdjust1.setDirection(Servo.Direction.REVERSE);  // set if necessary
    }

    public void setServoAdjust(double position){
        ServoAdjust1.setPosition(position);
        ServoAdjust2.setPosition(position);
    }
}
