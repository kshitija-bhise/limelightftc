package org.firstinspires.ftc.teamcode.Vision;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.followerConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.localizerConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Vision.DistanceEstimator;

@Autonomous(name = "strafe")
public class AlignmentLR extends LinearOpMode {

    private DcMotorEx RF, RR, LF, LR;
    private Limelight3A limelight;
    private DistanceEstimator distanceEstimator;
    private Follower follower;

    private final double target_distance = 30;
    private final double dist_tolerance = 1.0;
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

        waitForStart();

        while (opModeIsActive()){

            if (!distanceEstimator.hasTarget()) {
                telemetry.addData("Status", "No Target");
                stopRobot();
                telemetry.update();
                continue;
            }

            double tx = distanceEstimator.getTx();  // horizontal offset
            double strafePower = kP_strafe * tx;

            // Clamp power
            strafePower = Math.max(-0.5, Math.min(0.5, strafePower));

            moveStrafe(strafePower);

            telemetry.addData("Tx", tx);
            telemetry.addData("Strafe Power", strafePower);
            telemetry.update();

        }

    }
    public void moveStrafe(double power){
        LF.setPower(power);
        RR.setPower(power);
        RF.setPower(-power);
        LR.setPower(-power);

    }

    public void stopRobot() {
        RF.setPower(0);
        RR.setPower(0);
        LF.setPower(0);
        LR.setPower(0);
    }
}