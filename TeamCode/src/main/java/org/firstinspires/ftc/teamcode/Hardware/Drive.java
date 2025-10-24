package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Drive {
    private DcMotorEx LF, RF, LR, RR;

    // Constructor — initializes the motors
    public Drive(DcMotorEx LF, DcMotorEx RF, DcMotorEx LR, DcMotorEx RR) {
        this.LF = LF;
        this.RF = RF;
        this.LR = LR;
        this.RR = RR;

        LR.setDirection(DcMotorSimple.Direction.REVERSE);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // Drive method
    public void drive(double forward, double strafe, double turn) {
        final double DEAD = 0.05;

        // Apply deadband
        forward = (Math.abs(forward) < DEAD) ? 0 : forward;
        strafe  = (Math.abs(strafe)  < DEAD) ? 0 : strafe;
        turn    = (Math.abs(turn)    < DEAD) ? 0 : turn;

        // Calculate wheel powers
        double flPower = forward + strafe + turn;
        double frPower = forward - strafe - turn;
        double blPower = forward - strafe + turn;
        double brPower = forward + strafe - turn;

        // Normalize (optional but recommended)
        double max = Math.max(Math.abs(flPower),
                Math.max(Math.abs(frPower), Math.max(Math.abs(blPower), Math.abs(brPower))));
        if (max > 1.0) {
            flPower /= max;
            frPower /= max;
            blPower /= max;
            brPower /= max;
        }

        // Set power to motors
        LF.setPower(flPower);
        RF.setPower(frPower);
        LR.setPower(blPower);
        RR.setPower(brPower);
    }
}
