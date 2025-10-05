package org.firstinspires.ftc.teamcode.Vision;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.followerConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.localizerConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Autonomous(name = "Final Alignment")
public class Align extends LinearOpMode {

    private Limelight3A limelight;

    private DistanceEstimator distanceEstimator;
    private Follower follower;
    private DcMotorEx RF, RR, LF, LR;

    private final double target_distance = 30;
    private final double dist_tolerance = 1.0;
    private final double strafe_tolerance = 1.0;
    private final double turn_tolerance = 1.0;

    private final double kP_turn = 0.03;
    private final double kP_forward = 0.05;
    private final double kP_strafe = 0.03;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        RF = hardwareMap.get(DcMotorEx.class, "RF");


        RR = hardwareMap.get(DcMotorEx.class, "RR");
        LF = hardwareMap.get(DcMotorEx.class, "LF");
        LR = hardwareMap.get(DcMotorEx.class, "LR");

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LR.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight.pipelineSwitch(6);
        limelight.start();

        distanceEstimator = new DistanceEstimator(limelight, 32.0, 6.0, 27.0);

        PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
        follower = new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();

        telemetry.setMsTransmissionInterval(11);
        waitForStart();

        while (opModeIsActive()) {

            if (!distanceEstimator.hasTarget()) {
                telemetry.addData("Status", "No target");
                stopRobot();
                telemetry.update();
                continue;
            }

            LLResult result = limelight.getLatestResult();
            if (result == null || !result.isValid()) {
                telemetry.addData("Status", "No data available");
                stopRobot();
                telemetry.update();
                continue;
            }

            double tx = distanceEstimator.getTx();                  // strafe
            double distance = distanceEstimator.getDistanceInches(); // forward/back
            Pose3D botpose = result.getBotpose();
            double yaw = botpose.getOrientation().getYaw();

            double forwardPower = 0, strafePower = 0, turnPower = 0;

            boolean forwardAligned = Math.abs(distance - target_distance) <= dist_tolerance;
            boolean strafeAligned  = Math.abs(tx) <= strafe_tolerance;
            boolean turnAligned    = Math.abs(yaw + 125) <= turn_tolerance;

            if (!forwardAligned) {
                forwardPower = kP_forward * (distance - target_distance);
                forwardPower = Math.max(-0.5, Math.min(0.5, forwardPower));
            }

            if (!strafeAligned) {
                strafePower = kP_strafe * tx;
                strafePower = Math.max(-0.5, Math.min(0.5, strafePower));
            }

            if (!turnAligned) {
                turnPower = kP_turn * (yaw + 125);
                turnPower = Math.max(-0.5, Math.min(0.5, turnPower));
            }

//            follower.setTeleOpDrive(forwardPower,strafePower,turnPower, true);

            move(forwardPower, strafePower, turnPower);

            telemetry.addData("Status", forwardAligned && strafeAligned && turnAligned ? "Fully Aligned" : "Aligning...");
            telemetry.addData("Distance", distance);
            telemetry.addData("Tx", tx);
            telemetry.addData("Yaw", yaw);
            telemetry.addData("Forward Power", forwardPower);
            telemetry.addData("Strafe Power", strafePower);
            telemetry.addData("Turn Power", turnPower);
            telemetry.update();
        }
    }

    public void move(double forwardPower, double strafePower, double turnPower) {
        double LFPower = forwardPower + strafePower + turnPower;
        double RFPower = forwardPower - strafePower - turnPower;
        double LRPower = forwardPower - strafePower + turnPower;
        double RRPower = forwardPower + strafePower - turnPower;

        double max = Math.max(Math.abs(LFPower), Math.max(Math.abs(RFPower),
                Math.max(Math.abs(LRPower), Math.abs(RRPower))));
        if (max > 1.0) {
            LFPower /= max;
            RFPower /= max;
            LRPower /= max;
            RRPower /= max;
        }

        LF.setPower(LFPower);
        RF.setPower(RFPower);
        LR.setPower(LRPower);
        RR.setPower(RRPower);
    }

    public void stopRobot() {
        LF.setPower(0);
        RF.setPower(0);
        LR.setPower(0);
        RR.setPower(0);
    }
}
