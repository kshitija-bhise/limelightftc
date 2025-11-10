package org.firstinspires.ftc.teamcode.Hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.Wait;

@Configurable
public class ActiveIntake {
    public DcMotorEx Intake;
    public static double slowIntakeSpeed = 0.5;
    public static int rollBackTicks = 50;
    public static double rollBackPower = 0.8;
    public ActiveIntake(HardwareMap hardwareMap){
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void startIntake(){
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setPower(0.8);
    }

    public void startOuttake(){
        Intake.setPower(-1.0);
    }
    public void stopIntake(){
        Intake.setPower(0.0);
    }

    public void slowIntake(){
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setPower(slowIntakeSpeed);
    }
    public void rotateBack(){
        // this we did because whiie shooting our last ball is get till back to shoot
        stopIntake();
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Wait.mySleep(500);
        int currentTicks = Intake.getCurrentPosition();
        Intake.setTargetPosition(currentTicks - rollBackTicks);
        Intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Intake.setPower(rollBackPower);
        while(Intake.isBusy());
        Wait.mySleep(500);

    }
}
