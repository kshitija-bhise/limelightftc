package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.Drive;
import org.firstinspires.ftc.teamcode.Hardware.Shooter;

@TeleOp
public class TeleOpFull extends LinearOpMode {
    public DcMotorEx RF, LF, RR, LR;
    public Shooter shooter;

    @Override
    public void runOpMode() throws InterruptedException {
        Shooter shooter = new Shooter(hardwareMap);
        CameraAlign cameraAlign = new CameraAlign(hardwareMap);

        RF = hardwareMap.get(DcMotorEx.class, "RF");
        RR = hardwareMap.get(DcMotorEx.class, "RR");
        LF = hardwareMap.get(DcMotorEx.class, "LF");
        LR = hardwareMap.get(DcMotorEx.class, "LR");

        LR.setDirection(DcMotorSimple.Direction.REVERSE);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.setMsTransmissionInterval(11);

        waitForStart();

        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y;
            double strafe  = gamepad1.left_stick_x;
            double turn    = gamepad1.right_stick_x;

            if (gamepad1.b) {
                cameraAlign.Align(forward, strafe, turn);
            } else {
                drive(forward, strafe, turn);
            }

            if (gamepad1.dpad_up){
                shooter.startShooter();
            }else shooter.stopShooter();

            if(gamepad1.x){
                shooter.shoot();
                sleep(500);
                shooter.resetServo();
            }
        }
    }

    public void drive(double forward, double strafe, double turn) {
        final double DEAD = 0.05;

        forward = (Math.abs(forward) < DEAD) ? 0 : forward;
        strafe  = (Math.abs(strafe)  < DEAD) ? 0 : strafe;
        turn    = (Math.abs(turn)    < DEAD) ? 0 : turn;

        double flPower = forward + strafe + turn;
        double frPower = forward - strafe - turn;
        double blPower = forward - strafe + turn;
        double brPower = forward + strafe - turn;
        LF.setPower(flPower);
        RF.setPower(frPower);
        LR.setPower(blPower);
        RR.setPower(brPower);

    }

    public void StopMotors(){
        LF.setPower(0);
        LR.setPower(0);
        RF.setPower(0);
        RR.setPower(0);
    }



}
