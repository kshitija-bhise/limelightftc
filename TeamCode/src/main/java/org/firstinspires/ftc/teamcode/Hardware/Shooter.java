package org.firstinspires.ftc.teamcode.Hardware;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Shooter {
    private final DcMotorEx S1;
    private final DcMotorEx S2;
    private final Servo Push;

    public Shooter(HardwareMap hardwareMap){
        S1 = hardwareMap.get(DcMotorEx.class, "S1");
        S2 = hardwareMap.get(DcMotorEx.class, "S2");
        Push = hardwareMap.get(Servo.class,"Push");

        S1.setDirection(DcMotorEx.Direction.REVERSE);

        S1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        S2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        S1.setVelocityPIDFCoefficients(20, 1, 20, 5);
        S2.setVelocityPIDFCoefficients(20, 1, 20, 5);

    }

    public void startShooter(){
        S1.setVelocity(1800);
        S2.setVelocity(1800);
    }
    public void stopShooter(){
        S1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        S2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        S1.setPower(0);
        S2.setPower(0);
    }

    public void shoot(){
        if (S1.getVelocity()>1500){
            Push.setPosition(0.4);
        }
    }

    public void resetServo(){
        Push.setPosition(0);
    }
}


