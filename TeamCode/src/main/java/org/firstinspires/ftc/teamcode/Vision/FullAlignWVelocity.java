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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Disabled
@Autonomous(name = "Final Alignment")
public class FullAlignWVelocity extends LinearOpMode {

    private Limelight3A limelight;

    private DistanceEstimator distanceEstimator;
    private Follower follower;
    private DcMotorEx RF, RR, LF, LR;

    private final double target_distance = 50;
    private final double dist_tolerance = 1.0;
    private final double strafe_tolerance = 1.0;
    private final double turn_tolerance = 1.0;

    private final double kP_turn = 0.03;
    private final double kP_forward = 0.05;
    private final double kP_strafe = 0.05;



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

        distanceEstimator = new DistanceEstimator(limelight, 18.0, 10.8, 29.0);

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

            double forwardVel = 0, strafeVel = 0, turnVel = 0;

            boolean forwardAligned = Math.abs(distance - target_distance) <= dist_tolerance;
            boolean strafeAligned  = Math.abs(tx) <= strafe_tolerance;
            boolean turnAligned    = Math.abs(yaw + 125) <= turn_tolerance;

            if (!forwardAligned) {
                forwardVel = kP_forward * (distance - target_distance);
                forwardVel = Math.max(-50, Math.min(50, forwardVel));
            }

            if (!strafeAligned) {
                strafeVel = kP_strafe * tx;
                strafeVel = Math.max(-50, Math.min(50, strafeVel));
            }

            if (!turnAligned) {
                turnVel = kP_turn * (yaw + 125);
                turnVel = Math.max(-50, Math.min(50, turnVel));
            }
            if (forwardAligned && strafeAligned && turnAligned) {
                stopRobot();
            }

            move(forwardVel, strafeVel, turnVel);

            telemetry.addData("Status", forwardAligned && strafeAligned && turnAligned ? "Fully Aligned" : "Aligning...");
            telemetry.addData("Distance", distance);
            telemetry.addData("Tx", tx);
            telemetry.addData("Yaw", yaw);
            telemetry.addData("Forward Power", forwardVel);
            telemetry.addData("Strafe Power", strafeVel);
            telemetry.addData("Turn Power", turnVel);
            telemetry.update();
        }
    }


    public void move(double forwardVel, double strafeVel, double turnVel){
        double LF_inchPerSec = forwardVel + strafeVel + turnVel;
        double RF_inchPerSec = forwardVel - strafeVel - turnVel;
        double LR_inchPerSec = forwardVel - strafeVel + turnVel;
        double RR_inchPerSec = forwardVel + strafeVel - turnVel;

        double Gear_Ratio = 1.85;
        double Ticks_Per_Rev = 384.5;
        double Wheel_circumference = 4.094 * Math.PI;

        double LF_TPS = (LF_inchPerSec / Wheel_circumference) * Ticks_Per_Rev * Gear_Ratio;
        double RF_TPS = (RF_inchPerSec / Wheel_circumference) * Ticks_Per_Rev * Gear_Ratio;
        double LR_TPS = (LR_inchPerSec / Wheel_circumference) * Ticks_Per_Rev * Gear_Ratio;
        double RR_TPS = (RR_inchPerSec / Wheel_circumference) * Ticks_Per_Rev * Gear_Ratio;

        LF.setVelocity(LF_TPS);
        RF.setVelocity(RF_TPS);
        LR.setVelocity(LR_TPS);
        RR.setVelocity(RR_TPS);
    }

    public void stopRobot() {
        LF.setVelocity(0);
        RF.setVelocity(0);
        LR.setVelocity(0);
        RR.setVelocity(0);
    }
}
