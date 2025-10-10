package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoAdjust {
    private Servo shooterAligner;
    private double dist_max;
    private double dist_min;
    private double servo_min;
    private double servo_max;

    public ServoAdjust(Servo shooterAligner, double dist_max, double dist_min, double servo_min, double servo_max){
        this.shooterAligner = shooterAligner;
        this.dist_max = dist_max;
        this.dist_min = dist_min;
        this.servo_min = servo_min;
        this.servo_max = servo_max;
    }

    public void setServoAdjust(double current_distance){
        if(current_distance >= dist_min && current_distance <= dist_max){
            double servoPos = servo_max - (current_distance - dist_min) * (servo_max - servo_min) / (dist_max - dist_min);
            shooterAligner.setPosition(servoPos);

        }
        else {
            shooterAligner.setPosition(servo_max);
        }
    }


}