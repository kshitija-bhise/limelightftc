package org.firstinspires.ftc.teamcode.pedroPathing.paths;

import android.view.accessibility.AccessibilityRecord;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTag;
import org.firstinspires.ftc.teamcode.Hardware.ActiveIntake;
import org.firstinspires.ftc.teamcode.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.Teleop.CameraAlign;
import org.firstinspires.ftc.teamcode.Vision.DistanceEstimator;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class BlueAlliance extends LinearOpMode {
    Follower follower;
    ElapsedTime timer;
    Shooter shooter;
    CameraAlign cameraAlign;
    ActiveIntake intake;
    DistanceEstimator distanceEstimator;
    boolean shoot;

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
                        new BezierLine(new Pose(34.105, 136.211), new Pose(60.000, 85.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .addPath(
                        // Collect
                        new BezierLine(new Pose(60.000, 85.000), new Pose(14.470, 84.470))
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Score
                        new BezierLine(new Pose(14.470, 84.470), new Pose(60.000, 85.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .addPath(
                        // Path 4
                        new BezierLine(new Pose(60.000, 85.000), new Pose(44.211, 60.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .addPath(
                        // Path 5
                        new BezierLine(new Pose(44.211, 60.000), new Pose(13.520, 60.211))
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Path 6
                        new BezierLine(new Pose(13.520, 60.211), new Pose(60.000, 85.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .addPath(
                        // Path 7
                        new BezierLine(new Pose(60.000, 85.000), new Pose(43.579, 35.579))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .addPath(
                        // Path 8
                        new BezierLine(new Pose(43.579, 35.579), new Pose(13.474, 36.211))
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Path 9
                        new BezierLine(new Pose(13.474, 36.211), new Pose(60.000, 85.000))
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
                        intake.stopIntake();
                        break;
                    case 2:
                        autoShoot();
                        break;
                    case 3:
                        break;
                    case 4:
                        intake.stopIntake();
                        break;
                    case 5:
                        autoShoot();
                        break;
                    case 7:
                        intake.stopIntake();
                        break;
                    case 8:
                        autoShoot();
                        break;

                }
                state++;
                if (state == 4 || state == 7) {
                    follower.followPath(paths.getPath(state), false);
                } else {
                    sleep((state == 0) ? 0 : 2000);
                    follower.followPath(paths.getPath(state));
                }
            }
            follower.update();
        }
    }

    public void continueShoot() {
        for (int i = 0; i < 3; i++) {
            intake.stopIntake();
            while (!shooter.shoot()) shooter.startShooter();
            sleep(500);
            shooter.resetServo();
            intake.startIntake();
            sleep(i == 2 ? 0 : 2000);
        }
    }

    public void autoShoot() {
        timer.reset();
        while (timer.seconds() < 2) {
            cameraAlign.Align(0, 0, 0);
            shooter.startShooter();
        }
        continueShoot();
        shooter.stopShooter();
        intake.startIntake();
    }
}


