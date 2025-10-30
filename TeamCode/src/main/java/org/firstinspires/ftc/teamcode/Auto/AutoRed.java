package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.followerConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.localizerConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.paths.BlueAlliance;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.BlueAllianceCorner;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.RedAllianceCorner;

@Autonomous(name = "RedAllianceCorner")
public class AutoRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

        Follower follower = new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();


        follower.setPose(BlueAllianceCorner.start);

        waitForStart();

        if (opModeIsActive()) {
            // comes a little bit forward to launch first 3 artifacts
            follower.followPath(RedAllianceCorner.launch1());
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
            }
            // sleep will be replaced by the action of the launcher
            sleep(2000);

            // next path is align with the first set of the artifacts
            follower.followPath(RedAllianceCorner.Collection());
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
            }
            sleep(2000);
            follower.followPath(RedAllianceCorner.Collect());
            while (opModeIsActive()&&follower.isBusy()){
                follower.update();
            }
            sleep(2000);
//            follower.followPath(RedAllianceCorner.Launch2());
//            while (opModeIsActive()&&follower.isBusy()){
//                while (opModeIsActive()&&follower.isBusy()){
//                    follower.update();
//                }
            }
        }

    }

//
//
// 1st November 2025
//public class GeneratedPaths {
//
//    public static PathBuilder builder = new PathBuilder();
//
//    public static PathChain ScorePose = builder
//            .addPath(
//                    new BezierLine(new Pose(28.500, 128.000), new Pose(60.000, 85.000))
//            )
//            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
//            .build();
//
//    public static PathChain pickup1pose = builder
//            .addPath(new BezierLine(new Pose(60.000, 85.000), new Pose(14.471, 84.455)))
//            .setTangentHeadingInterpolation()
//            .build();
//
//    public static PathChain ScorePose = builder
//            .addPath(new BezierLine(new Pose(14.471, 84.455), new Pose(60.000, 85.000)))
//            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
//            .build();
//
//    public static PathChain pickup2pose = builder
//            .addPath(
//                    new BezierCurve(
//                            new Pose(60.000, 85.000),
//                            new Pose(73.068, 55.038),
//                            new Pose(13.522, 59.783)
//                    )
//            )
//            .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
//            .build();
//
//    public static PathChain ScorePose = builder
//            .addPath(new BezierLine(new Pose(13.522, 59.783), new Pose(60.000, 85.000)))
//            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
//            .build();
//
//    public static PathChain pickup3pose = builder
//            .addPath(
//                    new BezierCurve(
//                            new Pose(60.000, 85.000),
//                            new Pose(70.932, 33.687),
//                            new Pose(15.183, 36.059)
//                    )
//            )
//            .setTangentHeadingInterpolation()
//            .build();
//
//    public static PathChain ScorePose = builder
//            .addPath(new BezierLine(new Pose(15.183, 36.059), new Pose(60.000, 85.000)))
//            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
//            .build();
//}
//
