package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.ActiveIntake;
import org.firstinspires.ftc.teamcode.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.Teleop.CameraAlign;
import org.firstinspires.ftc.teamcode.Vision.DistanceEstimator;

@Autonomous(name = "Shooting")
public class Shooting extends LinearOpMode {
    ElapsedTime timer;
    Shooter shooter;
    CameraAlign cameraAlign;
    ActiveIntake intake;
    DistanceEstimator distanceEstimator;

    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime();
        shooter = new Shooter(hardwareMap);
        cameraAlign = new CameraAlign(hardwareMap);
        intake = new ActiveIntake(hardwareMap);
        distanceEstimator = new DistanceEstimator(hardwareMap.get(Limelight3A.class, "limelight"), 19, 11, 29.5);
        waitForStart();
        timer.reset();
        while (timer.seconds() < 2) {
            cameraAlign.Align(0, 0, 0);
        }
        while (opModeIsActive()) {
            timer.reset();
            while (timer.seconds() < 2 && opModeIsActive()) {
                shooter.startShooter();
                intake.startIntake();
                display();
            }
            intake.stopIntake();
            boolean shoot = shooter.shoot();
            telemetry.addData("Shoot", shoot);
            telemetry.update();
            if (shoot) {
                timer.reset();
                while (timer.milliseconds() < 500 && opModeIsActive()) display();
                timer.reset();
                shooter.resetServo();
                while (timer.milliseconds() < 500 && opModeIsActive()) display();
            }
            display();
        }
    }

    public void display() {
        telemetry.addData("Shooter RPM", shooter.getShooterVelocity());
        telemetry.addData("Distance", distanceEstimator.getDistanceInches());
        telemetry.update();
    }
}
