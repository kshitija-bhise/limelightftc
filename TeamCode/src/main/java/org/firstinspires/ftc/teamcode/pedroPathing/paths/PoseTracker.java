package org.firstinspires.ftc.teamcode.pedroPathing.paths;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.followerConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.localizerConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@Autonomous(name = "Pose Tracker")
public class PoseTracker extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

        follower = new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();


        waitForStart();

        while (opModeIsActive()) {

            follower.updatePose();

            Pose pose = follower.getPose();
            double x = pose.getX();
            double y = pose.getY();
            double heading = pose.getHeading();

            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("heading", heading);
            telemetry.update();
        }
    }
}
