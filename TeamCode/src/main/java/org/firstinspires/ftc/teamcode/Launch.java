package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class Launch extends LinearOpMode {
    public DcMotorEx Shoot;

    @Override
    public void runOpMode() throws InterruptedException {
        Shoot = hardwareMap.get(DcMotorEx.class,"Shoot");

        Shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Shoot.setVelocityPIDFCoefficients(20,1,20,5);

        double velo = 1000;

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.dpad_up){
                Shoot.setVelocity(velo);
            }else{
                Shoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Shoot.setPower(0);
            }
        }


    }
}
