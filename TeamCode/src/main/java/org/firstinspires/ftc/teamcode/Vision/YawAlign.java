//package org.firstinspires.ftc.teamcode.Vision;
//
//import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;
//import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.followerConstants;
//import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.localizerConstants;
//import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.stopRobot;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.ftc.FollowerBuilder;
//import com.pedropathing.paths.PathConstraints;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//
//@Autonomous(name = "YawAlign")
//public class YawAlign extends LinearOpMode {
//
//    public Limelight3A limelight;
//    public DcMotorEx RF, LF, RR, LR;
//    public DistanceEstimator distanceEstimator;
//    public Follower follower;
//    private final double kP = 0.03;
//    private final double target_distance = 30;
//    private final double dist_tolerance = 1.0;
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
//        limelight.pipelineSwitch(6);
//        limelight.start();
//
//        distanceEstimator = new DistanceEstimator(limelight, 32.0, 6.0, 27.0);
//
//        PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
//        follower = new FollowerBuilder(followerConstants, hardwareMap)
//                .pathConstraints(pathConstraints)
//                .mecanumDrivetrain(driveConstants)
//                .pinpointLocalizer(localizerConstants)
//                .build();
//
//            telemetry.setMsTransmissionInterval(11);
//
//        limelight.pipelineSwitch(6);
//
//        limelight.start();
//
//        waitForStart();
//
//        while (opModeIsActive()){
//
//            LLResult result = limelight.getLatestResult();
//
//            if (!distanceEstimator.hasTarget()) {
//                telemetry.addData("Status", "No Target");
//                stopRobot();
//                telemetry.update();
//                continue;
//            }
//
//            if (result.isValid()){
//                Pose3D botpose = result.getBotpose();
//                double Yaw = botpose.getOrientation().getYaw();
//                double final_yaw = Yaw + 125;
//
//                double turnError = final_yaw;
//
//                double turnPower = kP * turnError;
//
//                turnPower = Math.max(-0.5, Math.min(0.5, turnPower));
//
//                // Stop turning if close enough
//                if (Math.abs(turnError) < 1) {
//                    turnPower = 0;
//                }
//
//                turn(turnPower);
//
//                telemetry.addData("Turn Error", turnError);
//                telemetry.addData("Turn Power", turnPower);
//                telemetry.update();
//
//
//            } else {
//                telemetry.addData("Limelight", "No data available");
//                telemetry.update();
//            }
//        }
//    }
//    public void turn(double power){
//        RF.setPower(-power);
//        RR.setPower(-power);
//        LF.setPower(power);
//        LR.setPower(power);
//    }
//
//    public void stopRobot(){
//        RF.setPower(0);
//        RR.setPower(0);
//        LF.setPower(0);
//        LR.setPower(0);
//    }
//}
