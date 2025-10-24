package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.Shooter;

@TeleOp
public class AngleKpKD extends OpMode {
    private Limelight3A limelight;
    private DcMotorEx RF, LF, RR, LR;


    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        RF = hardwareMap.get(DcMotorEx.class, "RF");
        RR = hardwareMap.get(DcMotorEx.class, "RR");
        LF = hardwareMap.get(DcMotorEx.class, "LF");
        LR = hardwareMap.get(DcMotorEx.class, "LR");

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LR.setDirection(DcMotorSimple.Direction.REVERSE);

        LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(6);

        limelight.start();
    }

    @Override
    public void loop() {
        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();

        double previousTx = 0;
        double previousTime = 0;

        if (gamepad1.b) {
            if (result.isValid()) {
                double tx = result.getTx();
                telemetry.addData("TX", tx);

                double kP = 0.05;
                double kD = 0.08;
                double dist_tolerance = 1.0;

                double currentTime = System.currentTimeMillis() / 1000.0; // seconds
                double deltaTime = currentTime - previousTime;
                double derivative = 0;

                if (deltaTime > 0) {
                    derivative = (tx - previousTx) / deltaTime;
                }

                double turnPower = (kP * tx) + (kD * derivative);

                turnPower = Math.max(-0.6, Math.min(turnPower, 0.6));

                if (Math.abs(tx) < dist_tolerance) {
                    StopMotors();
                    telemetry.addLine("Aligned!");
                } else {
                    drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, turnPower);
                }

                // Update previous values
                previousTx = tx;
                previousTime = currentTime;

                telemetry.addData("TxNC", tx);
                telemetry.addData("TurnPower", turnPower);
                telemetry.addData("Derivative", derivative);
                telemetry.update();
            }
        }
        drive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );
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
