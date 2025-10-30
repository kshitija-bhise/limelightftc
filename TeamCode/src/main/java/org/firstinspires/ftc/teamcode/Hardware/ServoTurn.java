//package org.firstinspires.ftc.teamcode.Hardware;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//
//public class ServoTurn {
//    private final Servo servoTurn;
//    private double currentPosition = 0.0;
//    private final double[] positions = {0.0, 0.35, 0.7}; // your fixed positions
//    private int currentIndex = 0;
//
//    public ServoTurn(HardwareMap hardwareMap) {
//        servoTurn = hardwareMap.get(Servo.class, "ServoTurn");
//        servoTurn.setPosition(currentPosition);
//    }
//
//    public void setCollect() {
//        currentIndex++;
//        if (currentIndex >= positions.length) {
//            currentIndex = 0; // loop back to start
//        }
//        servoTurn.setPosition(positions[currentIndex]);
//    }
//
//    public void setShoot() {
//        currentIndex--;
//        if (currentIndex < 0) {
//            currentIndex = positions.length - 1; // loop to last
//        }
//        servoTurn.setPosition(positions[currentIndex]);
//    }
//
//    public double getPosition() {
//        return positions[currentIndex];
//    }
//}
