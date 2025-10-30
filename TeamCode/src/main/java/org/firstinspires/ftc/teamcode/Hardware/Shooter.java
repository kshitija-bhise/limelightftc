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
    private final double shooterVelocity = 1600;

    public Shooter(HardwareMap hardwareMap) {
        S1 = hardwareMap.get(DcMotorEx.class, "S1");
        S2 = hardwareMap.get(DcMotorEx.class, "S2");
        Push = hardwareMap.get(Servo.class, "Push");
        resetServo();
        S1.setDirection(DcMotorEx.Direction.REVERSE);

        S1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        S2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        S1.setVelocityPIDFCoefficients(300, 15, 30, 5);
        S2.setVelocityPIDFCoefficients(300, 15, 30, 5);

    }

    public void startShooter() {
        S1.setVelocity(shooterVelocity);
        S2.setVelocity(shooterVelocity);
    }

    public void stopShooter() {
        S1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        S2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        S1.setPower(0);
        S2.setPower(0);
    }

    public double getShooterVelocity() {
        return S1.getVelocity();
    }

    public boolean shoot() {
        if (S1.getVelocity() < shooterVelocity + 25 && S1.getVelocity() > shooterVelocity - 25) {
            Push.setPosition(0.4);
            return true;
        }
        return false;
    }

    public void resetServo() {
        Push.setPosition(0.05);
    }
}


