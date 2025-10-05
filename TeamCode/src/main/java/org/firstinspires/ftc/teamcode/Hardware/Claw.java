package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Servo claw;

    // Constructor
    public Claw(Servo claw) {
        this.claw = claw;
    }

    public void open() {
        claw.setPosition(0.6);
    }

    public void close() {
        claw.setPosition(0.8);
    }
}
