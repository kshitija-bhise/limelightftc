package org.firstinspires.ftc.teamcode.Hardware;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Shooter {
    private DcMotorEx S1;
    private DcMotorEx S2;
    private Servo Push;

    public Shooter(HardwareMap hardwareMap){
        S1 = hardwareMap.get(DcMotorEx.class, "S1");
        S2 = hardwareMap.get(DcMotorEx.class, "S2");
        Push = hardwareMap.get(Servo.class,"Push");

        S1.setDirection(DcMotorEx.Direction.REVERSE);

        S1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        S1.setVelocityPIDFCoefficients(10.5, 0.0005, 12, 5);
    }
    public void startShooter(){
        S1.setVelocity(1500);
        S2.setVelocity(1500);
    }
    public void stopShooter(){
        S1.setVelocity(0);
        S2.setVelocity(0);
    }

    public void shoot(){
        if (S1.getVelocity()>1400){
            Push.setPosition(0.7);
        }
    }

    public void resetServo(){
        Push.setPosition(1);
    }
}


