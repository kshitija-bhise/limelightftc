package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TestAngle extends OpMode {
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

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(6);

        limelight.start();
    }

    @Override
    public void loop() {
        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();

        if(gamepad1.b) {
            if (result.isValid()) {
                double tx = result.getTx();
                telemetry.addData("TX", tx);
                telemetry.update();
                double kP = 0.05;
                double dist_tolerance = 1.0;

                double turnPower = kP * tx;

                turnPower = Math.max(-0.6, Math.min(turnPower, 0.6));

                if (Math.abs(tx) < dist_tolerance) {
                    StopMotors();
                    telemetry.addLine("Aligned!");
                } else {
                    drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, turnPower);
                }
                telemetry.addData("TxNC", tx);
                telemetry.addData("TurnPower", turnPower);
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


