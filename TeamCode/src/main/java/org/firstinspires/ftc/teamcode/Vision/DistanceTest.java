package org.firstinspires.ftc.teamcode.Vision;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.followerConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.localizerConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DistanceTest extends OpMode {
    private DistanceEstimator distanceEstimator;
    private Follower follower;
    private Limelight3A limelight;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(6);
        limelight.start();

        distanceEstimator = new DistanceEstimator(limelight, 18.0, 10.8, 29.0);

        PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

        follower = new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

    @Override
    public void loop() {
        if (!distanceEstimator.hasTarget()) {
            telemetry.addData("Status", "No Target");
            stop();
            telemetry.update();
            return;
        }

        double distance = distanceEstimator.getDistanceInches();
        double tx = distanceEstimator.getTx();
        double ty = distanceEstimator.getTy();

        telemetry.addData("Distance", distance);
        telemetry.addData("Tx", tx);
        telemetry.addData("Ty", ty);
        telemetry.update();
    }
}
