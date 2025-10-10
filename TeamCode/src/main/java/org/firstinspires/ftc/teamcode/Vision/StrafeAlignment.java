//package org.firstinspires.ftc.teamcode.Vision;
//
//import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;
//import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.followerConstants;
//import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.localizerConstants;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.ftc.FollowerBuilder;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathConstraints;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
//
//public class StrafeAlignment extends LinearOpMode {
//    private Limelight3A limelight;
//    private DistanceEstimator distanceEstimator;
//    private Follower follower;
//    private DcMotorEx RF, RR, LF, LR;
//    private IMU imu;
//    private final double target_distance = 30;
//    private final double dist_tolerance = 1.0;
//    private final double kP_strafe = 0.03;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//
//        RF = hardwareMap.get(DcMotorEx.class, "RF");
//        RR = hardwareMap.get(DcMotorEx.class, "RR");
//        LF = hardwareMap.get(DcMotorEx.class, "LF");
//        LR = hardwareMap.get(DcMotorEx.class, "LR");
//
//        LF.setDirection(DcMotorSimple.Direction.REVERSE);
//        LR.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        imu = hardwareMap.get(IMU.class, "imu");
//        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                RevHubOrientationOnRobot.UsbFacingDirection.DOWN);
//        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
//
//        limelight.pipelineSwitch(6);
//        limelight.start();
//
//        distanceEstimator = new DistanceEstimator(limelight, 32.0, 6.0, 27.0);
//
//        PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
//
//        follower = new FollowerBuilder(followerConstants, hardwareMap)
//                .pathConstraints(pathConstraints)
//                .mecanumDrivetrain(driveConstants)
//                .pinpointLocalizer(localizerConstants)
//                .build();
//
//        waitForStart();
//
//        while (opModeIsActive()){
//            follower.update();
//            Pose pose = follower.getPose();
//
//            telemetry.addData("X", pose.getX());
//            telemetry.addData("Y", pose.getY());
//            telemetry.addData("Heading", Math.toRadians(pose.getHeading()));
//            telemetry.update();
//
//            if (!distanceEstimator.hasTarget()){
//                telemetry.addData("Status", "No Target");
//                stop();
//                telemetry.update();
//                return;
//            }
//
//        }
//    }
//}
