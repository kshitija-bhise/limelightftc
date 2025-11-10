package org.firstinspires.ftc.teamcode.pedroPathing.paths;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.ActiveIntake;
import org.firstinspires.ftc.teamcode.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.Teleop.CameraAlign;
import org.firstinspires.ftc.teamcode.Vision.DistanceEstimator;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Configurable
@Autonomous
public class BlueAlliance extends LinearOpMode {

    Follower follower;
    ElapsedTime timer;
    Shooter shooter;
    CameraAlign cameraAlign;
    ActiveIntake intake;
    DistanceEstimator distanceEstimator;
    boolean shoot;
    public static double shooterSpeed = 1800;

    @Override
    public void runOpMode() throws InterruptedException {
        int state = -1;
        timer = new ElapsedTime();
        shooter = new Shooter(hardwareMap);
        cameraAlign = new CameraAlign(hardwareMap);
        intake = new ActiveIntake(hardwareMap);
        distanceEstimator = new DistanceEstimator(hardwareMap.get(Limelight3A.class, "limelight"), 19, 11, 29.5);

        shoot = false;

        follower = Constants.createFollower(hardwareMap);
        PathBuilder builder = new PathBuilder(follower);

        PathChain paths = builder
                .addPath(
                        // Score
                        new BezierLine(new Pose(34.105, 136.210), new Pose(45.000, 98.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .addPath(
                        // Collect
                        new BezierLine(new Pose(45.000, 98.000), new Pose(45.053, 83.789))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .addPath(
                        // grabPick
                        new BezierLine(new Pose(45.053, 83.789), new Pose(19, 83.579))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setGlobalDeceleration(1)
                .addPath(
                        // Score
                        new BezierLine(new Pose(19, 83.579), new Pose(45.000, 98.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .addPath(
                        // Path 4
                        new BezierLine(new Pose(45.000, 98.000), new Pose(44.211, 60.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .setGlobalDeceleration(1)
                .addPath(
                        // Path 5
                        new BezierLine(new Pose(44.211, 60.000), new Pose(15, 60.211))
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Path 6
                        new BezierLine(new Pose(15, 60.211), new Pose(45.000, 98.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .addPath(
                        // Path 7
                        new BezierLine(new Pose(45.000, 98.000), new Pose(43.579, 35.579))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .addPath(
                        // Path 8
                        new BezierLine(new Pose(43.579, 35.579), new Pose(13.474, 36.211))
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Path 9
                        new BezierLine(new Pose(13.474, 36.211), new Pose(45.000, 98.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();

            follower.setStartingPose(new Pose(34.105, 136.210, Math.toRadians(180)));
        waitForStart();

        while (opModeIsActive()) {
            if (!follower.isBusy()) {
                switch (state) {
                    case 0:
                        autoShoot();
                        break;
                    case 1:
                        intake.startIntake();
                        break;
                    case 2:
                        intake.slowIntake();
                        break;
                    case 3:
                        autoShoot();
                        break;
                    case 4:
                        intake.startIntake();
                        break;
                    case 5:
                        intake.slowIntake();
                        break;
                    case 6:
                        autoShoot();
                        break;
                    case 7:
                        intake.startIntake();
                        break;
                    case 8:
                        intake.slowIntake();
                        break;
                    case 9:
                        autoShoot();
                        break;
                }
                state++;   //increment of the state
                if (state == 4 || state == 7 || state == 1) {
                    follower.followPath(paths.getPath(state), false);  //runs in a continuous manner
                    follower.setMaxPower(0.9);
                } else {
                    if (!(state == 0 || state == 2 || state == 5 || state == 8 || state == 3)) {
                        sleep(500);
                    }
                    follower.setMaxPower(1);
                    follower.followPath(paths.getPath(state));
                }
            }else{
                switch (state) {
                    case 0:
                        shooter.startShooter();
                        break;
                    case 1:
                        break;
                    case 2:
                        break;
                    case 3:
                        shooter.startShooter();
                        break;
                    case 4:
                        break;
                    case 5:
                        break;
                    case 6:
                        shooter.startShooter();
                        break;
                    case 7:
                        break;
                    case 8:
                        break;
                    case 9:
                        shooter.startShooter();
                        break;
                }
            }
            follower.update();
        }

    }



    public void autoShoot() {
        timer.reset();
        while (timer.milliseconds() < 500) {
            //here alignment is missing
            shooter.startShooter();
        }
        shooter.continueShootThree(intake);
    }
}


