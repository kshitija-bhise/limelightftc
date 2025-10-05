package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class DistanceEstimator {
    private Limelight3A limelight;
    private double limelightAngleMounted;
    private double limelightLensHeight;
    private double goalHeightInches;

    public DistanceEstimator(Limelight3A limelight, double limelightAngleMounted, double limelightLensHeight, double goalHeightInches) {
        this.limelight = limelight;
        this.limelightAngleMounted = limelightAngleMounted;
        this.limelightLensHeight = limelightLensHeight;
        this.goalHeightInches = goalHeightInches;
    }

    public double getDistanceInches() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double angleToGoalDegrees = limelightAngleMounted + ty;
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);


            return (goalHeightInches - limelightLensHeight) / Math.tan(angleToGoalRadians);

        } else {
            return -1;
        }

    }

        public double getTx() {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                return result.getTx(); // horizontal offset (deg)
            }
            return 0;
        }

        public double getTy(){
        LLResult result = limelight.getLatestResult();
        if (result!= null && result.isValid()){
            return result.getTy(); // vertical offset (deg)
        }
        return 0;
        }

        public boolean hasTarget() {
            LLResult result = limelight.getLatestResult();
            return result != null && result.isValid();
        }

    public double getTxNc() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTxNC(); // horizontal offset (deg)
        }
        return 0;
    }

}
