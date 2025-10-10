//package org.firstinspires.ftc.teamcode.Auto;
//
//import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;
//import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.followerConstants;
//import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.localizerConstants;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.ftc.FollowerBuilder;
//import com.pedropathing.paths.PathConstraints;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import org.firstinspires.ftc.teamcode.pedroPathing.paths.RedAllianceCorner;
//
//@Autonomous(name = "RedAllianceCorner")
//public class AutoRed extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
//
//        Follower follower = new FollowerBuilder(followerConstants, hardwareMap)
//                .pathConstraints(pathConstraints)
//                .mecanumDrivetrain(driveConstants)
//                .pinpointLocalizer(localizerConstants)
//                .build();
//
//
//        follower.setPose(RedAllianceCorner.start);
//
//        waitForStart();
//
//        if (opModeIsActive()) {
//            // comes a little bit forward to launch first 3 artifacts
//            follower.followPath(RedAllianceCorner.launch1());
//            while (opModeIsActive() && follower.isBusy()) {
//                follower.update();
//            }
//            // sleep will be replaced by the action of the launcher
//            sleep(2000);
//
//            // next path is align with the first set of the artifacts
//            follower.followPath(RedAllianceCorner.Collection());
//            while (opModeIsActive() && follower.isBusy()) {
//                follower.update();
//            }
//            sleep(2000);
//            follower.followPath(RedAllianceCorner.Collect());
//            while (opModeIsActive()&&follower.isBusy()){
//                follower.update();
//            }
//            sleep(2000);
////            follower.followPath(RedAllianceCorner.Launch2());
////            while (opModeIsActive()&&follower.isBusy()){
////                while (opModeIsActive()&&follower.isBusy()){
////                    follower.update();
////                }
//            }
//        }
//
//    }
//
