//package org.firstinspires.ftc.teamcode.Vision;
//
//import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;
//import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.followerConstants;
//import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.localizerConstants;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.ftc.FollowerBuilder;
//import com.pedropathing.paths.PathConstraints;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//@Autonomous(name = "TxNC")
//public class Turn extends LinearOpMode {
//
//    public Limelight3A limelight;
//    private DistanceEstimator distanceEstimator;
//    private Follower follower;
//    private DcMotorEx RF, RR, LF, LR;
//    private final double target_distance = 30;
//    private final double dist_tolerance = 1.0;
//    private final double kP = 0.05 ;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
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
//        telemetry.setMsTransmissionInterval(11);
//        limelight.pipelineSwitch(6);
//
//        limelight.start();
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
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
//         while (opModeIsActive()){
//             if (!distanceEstimator.hasTarget()) {
//                 telemetry.addData("Status", "No Target");
//                 stop();
//                 telemetry.update();
//             }
//
////             double distance = distanceEstimator.getDistanceInches();
//             double TxNC = distanceEstimator.getTxNc();
//
//             double turnPower = kP * TxNC;
//
//             turnPower = Math.max(-0.5, Math.min(turnPower, 0.5));
//
//             if (Math.abs(TxNC) < dist_tolerance) {
//                 stopMotors();
//                 telemetry.addLine("Aligned!");
//             } else {
//                 turn(turnPower);
//             }
//
//             telemetry.addData("TxNC", TxNC);
//             telemetry.addData("TurnPower", turnPower);
//             telemetry.update();
//
//         }
//
//
//    }
//
//    public void turn(double power) {
//        LF.setPower(power);
//        LR.setPower(power);
//        RF.setPower(-power);
//        RR.setPower(-power);
//    }
//
//    public void stopMotors() {
//        LF.setPower(0);
//        LR.setPower(0);
//        RF.setPower(0);
//        RR.setPower(0);
//    }
//
//}
//
