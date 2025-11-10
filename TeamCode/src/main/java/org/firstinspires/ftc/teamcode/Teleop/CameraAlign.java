package org.firstinspires.ftc.teamcode.Teleop;

import android.view.inputmethod.EditorBoundsInfo;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.security.cert.CertificateRevokedException;

@Configurable
public class CameraAlign {
    private Limelight3A limelight;
    private DcMotorEx RF, LR, RR, LF;

    private double previousTx = 0;
    private double previousTime = 0;
    public static double kP = 0.04;
    public static double kD = 0.004 ;
    public static double maxPower = 0.45;
    public static  double dist_tolerance = 2;

    public CameraAlign(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        RF = hardwareMap.get(DcMotorEx.class, "RF");
        RR = hardwareMap.get(DcMotorEx.class, "RR");
        LF = hardwareMap.get(DcMotorEx.class, "LF");
        LR = hardwareMap.get(DcMotorEx.class, "LR");

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LR.setDirection(DcMotorSimple.Direction.REVERSE);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight.pipelineSwitch(6);
        limelight.start();
    }

    // ✅ Align method with forward, strafe, and turn
    public double alignTurnValue(double manualTurn) {
        LLResult result = limelight.getLatestResult();

        double turnPower = manualTurn; // default = driver control

        if (result.isValid()) {
            double tx = result.getTx();



            double currentTime = System.currentTimeMillis() / 1000.0;
            double deltaTime = currentTime - previousTime;
            double derivative = 0;

            if (deltaTime > 0) {
                derivative = (tx - previousTx) / deltaTime;
            }
//            if(tx < )
            // Add automatic correction to driver’s turn
            double autoTurn = (kP * tx) + (kD * derivative);
            autoTurn = Math.max(-maxPower, Math.min(autoTurn, maxPower));

            if (Math.abs(tx) < dist_tolerance) {
                autoTurn = 0; // stop turning when aligned
            }

            turnPower += autoTurn;

            previousTx = tx;
            previousTime = currentTime;
        }
        return turnPower;
    }

    private void drive(double forward, double strafe, double turn) {
        final double DEAD = 0.05;

        forward = (Math.abs(forward) < DEAD) ? 0 : forward;
        strafe = (Math.abs(strafe) < DEAD) ? 0 : strafe;
        turn = (Math.abs(turn) < DEAD) ? 0 : turn;

        double flPower = forward + strafe + turn;
        double frPower = forward - strafe - turn;
        double blPower = forward - strafe + turn;
        double brPower = forward + strafe - turn;

        LF.setPower(flPower);
        RF.setPower(frPower);
        LR.setPower(blPower);
        RR.setPower(brPower);
    }

    public void StopMotors() {
        LF.setPower(0);
        LR.setPower(0);
        RF.setPower(0);
        RR.setPower(0);
    }
}
