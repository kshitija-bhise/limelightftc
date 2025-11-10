package org.firstinspires.ftc.teamcode.Hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Util.Wait;

@Configurable
public class Shooter {
   public static double shooterVelocity = 2500;
   private final DcMotorEx S1;
   private final DcMotorEx S2;
   private final Servo Push;
   private final Servo LeftServo;
   private final Servo RightServo;
   public double angleServo = 0;
   public double previousDistance = 0;

   public Shooter(HardwareMap hardwareMap) {
      S1 = hardwareMap.get(DcMotorEx.class, "S1");
      S2 = hardwareMap.get(DcMotorEx.class, "S2");
      Push = hardwareMap.get(Servo.class, "Push");
      LeftServo = hardwareMap.get(Servo.class, "ServoAdjust1");
      RightServo = hardwareMap.get(Servo.class, "ServoAdjust2");
      LeftServo.setDirection(Servo.Direction.REVERSE);
      this.setAngle(0);

      resetServo();
      S1.setDirection(DcMotorEx.Direction.REVERSE);

      S1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      S2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        S1.setVelocityPIDFCoefficients(300, 15, 30, 5);
//        S2.setVelocityPIDFCoefficients(300, 15, 30, 5);

   }

   public void startShooter() {
      S1.setVelocity(shooterVelocity);
      S2.setVelocity(shooterVelocity);
   }

   public void stopShooter() {
      S1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      S2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

      S1.setPower(0);
      S2.setPower(0);
   }

   public void setShooter(double distance) {
      if (Math.abs(distance - previousDistance) > 5) {
         shooterVelocity = 6.059668 * distance + 1311.704630;
         angleServo = 0.00281404 * distance - 0.15172426;
         setAngle(angleServo);
      }
   }


   public double getShooterVelocity() {
      return S1.getVelocity();
   }

   public boolean shoot() {
      if (S1.getVelocity() < shooterVelocity + 25 && S1.getVelocity() > shooterVelocity - 25) {
         Push.setPosition(0.4);
         return true;
      }
      return false;
   }

   private void setAngle(double position) {
      LeftServo.setPosition(position);
      RightServo.setPosition(position);
   }


   public void resetServo() {
      Push.setPosition(0.05);
   }

   public void continueShootThree(ActiveIntake intake) {
      for (int i = 0; i < 3; i++) {
         intake.stopIntake();
         while (!this.shoot()) this.startShooter();
         Wait.mySleep(500);
         this.resetServo();
         if (i == 1) {
            intake.rotateBack();
            intake.startIntake();
         } else {
            intake.slowIntake();
         }

         intake.slowIntake();
         Wait.mySleep(i == 2 ? 0 : 2000);
      }
      this.stopShooter();
      intake.stopIntake();
   }
}


