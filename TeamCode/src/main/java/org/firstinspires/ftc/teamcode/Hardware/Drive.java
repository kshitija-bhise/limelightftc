package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drive {
    private DcMotor FL, FR, RL, RR;

    // Constructor
    public Drive(DcMotor FL, DcMotor FR, DcMotor RL, DcMotor RR) {
        this.FL = FL;
        this.FR = FR;
        this.RL = RL;
        this.RR = RR;
    }

    // Method to drive using gamepad input
    public void drive(Gamepad gamepad1) {
        double y = -gamepad1.left_stick_y;   // Forward/backward
        double x = gamepad1.left_stick_x;    // Strafe
        double rx = gamepad1.right_stick_x;  // Rotate

        double FL_power = y + x + rx;
        double FR_power = y - x - rx;
        double RL_power = y - x + rx;
        double RR_power = y + x - rx;

        // Normalize powers
        double max = Math.max(Math.abs(FL_power),
                Math.max(Math.abs(FR_power), Math.max(Math.abs(RL_power), Math.abs(RR_power))));

        if (max > 1.0) {
            FL_power /= max;
            FR_power /= max;
            RL_power /= max;
            RR_power /= max;
        }

        // Set motor powers
        FL.setPower(FL_power);
        FR.setPower(FR_power);
        RL.setPower(RL_power);
        RR.setPower(RR_power);

    }
}

