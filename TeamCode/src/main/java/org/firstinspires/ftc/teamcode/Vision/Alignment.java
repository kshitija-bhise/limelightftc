package org.firstinspires.ftc.teamcode.Vision;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.followerConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.localizerConstants;

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Disabled
@Autonomous(name = "Alignment")
public class Alignment extends OpMode {
    private Limelight3A limelight;
    private DistanceEstimator distanceEstimator;
    private Follower follower;
    private DcMotorEx RF, RR, LF, LR;
    private final double target_distance = 50;
    private final double dist_tolerance = 1.0;
    private final double kP = 0.05;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        RF = hardwareMap.get(DcMotorEx.class, "RF");
        RR = hardwareMap.get(DcMotorEx.class, "RR");
        LF = hardwareMap.get(DcMotorEx.class, "LF");
        LR = hardwareMap.get(DcMotorEx.class, "LR");

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LR.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight.pipelineSwitch(6);
        limelight.start();

        distanceEstimator = new DistanceEstimator(limelight, 11.0, 10.8, 29.0);

        PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

        follower = new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

    public void loop() {
        if (!distanceEstimator.hasTarget()) {
            telemetry.addData("Status", "No Target");
            stop();
            telemetry.update();
            return;
        }

        double distance = distanceEstimator.getDistanceInches();
        double tx = distanceEstimator.getTx();

        telemetry.addData("Distance", distance);
        telemetry.addData("Tx", tx);

        double error = distance - target_distance;

        // Check if within tolerance
        if (Math.abs(error) <= dist_tolerance) {
            stop();
            telemetry.addLine("Target distance reached");
        } else {
            double power = kP * error;

            power = Math.max(-0.5, Math.min(0.5, power));

            forward(power);
        }

        telemetry.update();
    }

    public void  forward(double power){
        RF.setPower(power);
        RR.setPower(power);
        LF.setPower(power);
        LR.setPower(power);
    }

    public void backward(double power){
        RF.setPower(-power);
        RR.setPower(-power);
        LF.setPower(-power);
        LR.setPower(-power);
    }

    public void stop(){
        RF.setPower(0);
        RR.setPower(0);
        LF.setPower(0);
        LR.setPower(0);
    }
}
