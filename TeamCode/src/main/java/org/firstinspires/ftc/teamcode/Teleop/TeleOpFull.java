package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.ActiveIntake;
import org.firstinspires.ftc.teamcode.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.Util.BuildPath;
import org.firstinspires.ftc.teamcode.Vision.DistanceEstimator;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class TeleOpFull extends LinearOpMode {
   public DcMotorEx RF, LF, RR, LR;
   ElapsedTime timer;
   Shooter shooter;
   CameraAlign cameraAlign;
   ActiveIntake intake;
   DistanceEstimator distanceEstimator;
   Follower follower;
   boolean shoot;
      boolean isTeleOpDriveStarted = false;

   @Override
   public void runOpMode() throws InterruptedException {
      timer = new ElapsedTime();
      shooter = new Shooter(hardwareMap);
      cameraAlign = new CameraAlign(hardwareMap);
      intake = new ActiveIntake(hardwareMap);
      distanceEstimator = new DistanceEstimator(hardwareMap.get(Limelight3A.class, "limelight"), 19, 11, 29.5);
      follower = Constants.createFollower(hardwareMap);
      shoot = false;


//      RF = hardwareMap.get(DcMotorEx.class, "RF");
//      RR = hardwareMap.get(DcMotorEx.class, "RR");
//      LF = hardwareMap.get(DcMotorEx.class, "LF");
//      LR = hardwareMap.get(DcMotorEx.class, "LR");
//
//      LR.setDirection(DcMotorSimple.Direction.REVERSE);
//      LF.setDirection(DcMotorSimple.Direction.REVERSE);

      telemetry.setMsTransmissionInterval(11);

      waitForStart();

      follower.startTeleopDrive(true);
      follower.update();
      while (opModeIsActive()) {

         follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
         follower.update();
         double forward = -gamepad1.left_stick_y;
         double strafe = gamepad1.left_stick_x;
         double turn = gamepad1.right_stick_x;


         if (gamepad1.bWasPressed()) {
            double error = distanceEstimator.getTx();
            follower.followPath(BuildPath.getTurnPath(follower.getPose(), Math.abs(error), error > 0));
            isTeleOpDriveStarted = false;
         } else if (gamepad1.b) {
            if ((!follower.isTurning() && !follower.isBusy()) && !isTeleOpDriveStarted) {
               follower.startTeleopDrive(true);
               isTeleOpDriveStarted = true;
            }
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, cameraAlign.alignTurnValue(gamepad1.right_stick_x), true);
         } else {
            boolean isGamePadActive = Math.abs(gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x) > 0.02;
            if (((!follower.isTurning() && !follower.isBusy()) || isGamePadActive) && !isTeleOpDriveStarted) {
               follower.startTeleopDrive(true);
               isTeleOpDriveStarted = true;
            }
         }

         if (distanceEstimator.getDistanceInches() != -1) {
            shooter.setShooter(distanceEstimator.getDistanceInches());
         }


         if (gamepad1.leftBumperWasPressed() && gamepad1.left_trigger > 0) {
            intake.rotateBack();
         } else if (gamepad1.left_bumper) {
            intake.startIntake();
         } else if (gamepad1.left_trigger > 0) {
            intake.slowIntake();
         } else {
            intake.stopIntake();
         }


         if (gamepad1.rightBumperWasPressed() && gamepad1.right_trigger > 0) {
            shooter.continueShootThree(intake);
         } else if (gamepad1.right_bumper) {
            shooter.startShooter();
         } else shooter.stopShooter();


         if (gamepad1.x) {
            intake.stopIntake();
            shoot = shooter.shoot();
            if (shoot) {
               timer.reset();
               while (timer.milliseconds() < 500 && opModeIsActive()) display();
               timer.reset();
               shooter.resetServo();
               while (timer.milliseconds() < 500 && opModeIsActive()) display();
            }
            display();
         }
         display();
      }
   }

//   public void drive(double forward, double strafe, double turn) {
//      final double DEAD = 0.05;
//
//      forward = (Math.abs(forward) < DEAD) ? 0 : forward;
//      strafe = (Math.abs(strafe) < DEAD) ? 0 : strafe;
//      turn = (Math.abs(turn) < DEAD) ? 0 : turn;
//
//      double flPower = forward + strafe + turn;
//      double frPower = forward - strafe - turn;
//      double blPower = forward - strafe + turn;
//      double brPower = forward + strafe - turn;
//      LF.setPower(flPower);
//      RF.setPower(frPower);
//      LR.setPower(blPower);
//      RR.setPower(brPower);
//
//   }

//   public void StopMotors() {
//      LF.setPower(0);
//      LR.setPower(0);
//      RF.setPower(0);
//      RR.setPower(0);
//   }

   public void display() {
      telemetry.addData("Shooter RPM", shooter.getShooterVelocity());
      telemetry.addData("Distance", distanceEstimator.getDistanceInches());
      telemetry.addData("Shoot", shoot);
      telemetry.addData("Tx", distanceEstimator.getTx());
      telemetry.addData("Pose", Math.toDegrees(follower.getHeading()));
      telemetry.addData("TeleOp", isTeleOpDriveStarted);
      telemetry.update();
   }
}
