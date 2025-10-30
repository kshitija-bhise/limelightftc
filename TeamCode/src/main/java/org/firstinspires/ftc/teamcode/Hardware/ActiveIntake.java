package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ActiveIntake {
    public DcMotorEx Intake;

    public ActiveIntake(HardwareMap hardwareMap){
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void startIntake(){
        Intake.setPower(1.0);
    }

    public void startOuttake(){
        Intake.setPower(-1.0);
    }
    public void stopIntake(){
        Intake.setPower(0.0);
    }

}
