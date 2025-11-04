package org.firstinspires.ftc.teamcode.Vision;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class test extends OpMode {

    private Limelight3A camera;
    private Follower follower;
    private boolean following = false;

    // Put your target location here (example: 50 inches forward, 30 inches left)
    private final Pose TARGET_LOCATION = new Pose(45, 98, Math.toRadians(135), FTCCoordinates.INSTANCE)
            .getAsCoordinateSystem(PedroCoordinates.INSTANCE);

    @Override
    public void init() {
        camera = hardwareMap.get(Limelight3A.class, "limelight");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(34, 136, Math.toRadians(180), FTCCoordinates.INSTANCE)
                .getAsCoordinateSystem(PedroCoordinates.INSTANCE));
    }

    @Override
    public void start() {
        camera.start();
    }

    @Override
    public void loop() {
        follower.update();

        // Follow a path to the target if not already following
//        if (!following) {
//            follower.followPath(
//                    follower.pathBuilder()
//                            .addPath(new BezierLine(follower.getPose(), TARGET_LOCATION))
//                            .setLinearHeadingInterpolation(
//                                    follower.getHeading(),
//                                    TARGET_LOCATION.minus(follower.getPose()).getAsVector().getTheta()
//                            )
//                            .build()
//            );
//            following = true;
//        }

        // Relocalize robot pose using MegaTag2 (AprilTag)
        Pose visionPose = getRobotPoseFromCamera();
        if (visionPose != null) {
            follower.setPose(visionPose);
        }

        if (following && !follower.isBusy()) following = false;

        telemetry.addData("Pose (Pedro)", follower.getPose());
        telemetry.update();
    }

    /**
     * Fetches robot pose from Limelight MegaTag 2 and converts to Pedro coordinates.
     */
    private Pose getRobotPoseFromCamera() {
        // Retrieve MegaTag 2 bot pose data (returns double[6]: x, y, z, roll, pitch, yaw)
        LLResult result = camera.getLatestResult();
        Pose3D botpose_mt2 = result.getBotpose_MT2();


        // If no valid data, return null
        if (botpose_mt2 == null) {
            return null;
        }

        // Limelight reports in meters, convert to inches
        double xInches = botpose_mt2.getPosition().x * 39.37; // field X (forward)
        double yInches = botpose_mt2.getPosition().y * 39.37; // field Y (left)
        double headingDeg = botpose_mt2.getOrientation().getYaw() * 180 / Math.PI;      // yaw in degrees
        double headingRad = Math.toRadians(headingDeg);

        // FTC pose
        Pose ftcPose = new Pose(xInches, yInches, headingRad, FTCCoordinates.INSTANCE);

        // Convert FTC coordinates to Pedro Pathing coordinates
        Pose pedroPose = ftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

        telemetry.addData("Limelight Pose (inches)", String.format("X: %.2f, Y: %.2f, H: %.2f°", xInches, yInches, headingDeg));
        telemetry.addData("Pedro Pose", pedroPose);

        return pedroPose;
    }
}
