package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.followerConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.localizerConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.BAC;

@Autonomous
public class TestAuto extends OpMode {

    PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    Follower follower = new FollowerBuilder(followerConstants, hardwareMap)
            .pathConstraints(pathConstraints)
            .mecanumDrivetrain(driveConstants)
            .pinpointLocalizer(localizerConstants)
            .build();

//    Shooter shooter = new Shooter(hardwareMap);
//    ActiveIntake intake = new ActiveIntake(hardwareMap);

    private Timer pathTimer, actionTimer, opmodeTimer;

    BAC BAC = new BAC();

    int PathState = 0;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower.setStartingPose(BAC.startPose);
    }

    @Override
    public void loop() {
        follower.update();
        automatedPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", PathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public void automatedPathUpdate(){
        switch (PathState){
            case 0:
//                shooter.startShooter();
                follower.followPath(BAC.scorePreload);
//                shooter.shoot();
                setPathState(1);
                break;
            case 1:
//                intake.startIntake();
                follower.followPath(BAC.grabPickup1);
                setPathState(2);
                break;
            case 2:
                if (!follower.isBusy());
//                intake.stopIntake();
//                shooter.startShooter();
                follower.followPath(BAC.scorePickup1);
//                shooter.shoot();
                setPathState(3);
                break;
            case 3:
                if (!follower.isBusy());
//                intake.startIntake();
                follower.followPath(BAC.grabPickup2);
                setPathState(4);
                break;
            case 4:
                if (!follower.isBusy());
//                intake.stopIntake();
//                shooter.startShooter();
                follower.followPath(BAC.scorePickup2);
//                shooter.shoot();
                setPathState(5);
                break;
            case 5:
                if (!follower.isBusy());
//                intake.startIntake();
                follower.followPath(BAC.grabPickup3);
                setPathState(6);
                break;
            case 6:
                if (!follower.isBusy());
//                intake.stopIntake();
//                shooter.startShooter();
                follower.followPath(BAC.scorePickup3);
//                shooter.shoot();
                setPathState(7);
                break;
            case 7:
                setPathState(-1);
                break;

        }

    }

    public void setPathState(int pState) {
        PathState = pState;
        pathTimer.resetTimer();
    }
}