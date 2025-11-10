package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Vision.DistanceEstimator;

import java.util.List;

@TeleOp(name = "Limelight Data Viewer", group = "Vision")
public class LimelightDataTeleOp extends LinearOpMode {
   private Limelight3A limelight;
   DistanceEstimator distanceEstimator;

   @Override
   public void runOpMode() {
      // Initialize Limelight
      limelight = hardwareMap.get(Limelight3A.class, "limelight");

      // Set pipeline to 1
      limelight.pipelineSwitch(7);

      limelight.start();

      telemetry.addLine("Limelight Initialized - Pipeline 1");
      telemetry.update();

      waitForStart();

      while (opModeIsActive()) {
         // Get latest vision result
         LLResult result = limelight.getLatestResult();
         if (result != null && result.isValid()) {
            telemetry.addLine("===== Limelight Data =====");
//            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
//            if (fiducials != null && !fiducials.isEmpty()) {
//               for (LLResultTypes.FiducialResult fid : fiducials) {
//                  int tagId = fid.getFiducialId();
//                  telemetry.addData("Tag ID", tagId);
//                  // You can pull other info to o:
//                  telemetry.addData("Tag X°", fid.getTargetXDegrees());
//                  telemetry.addData("Tag Y°", fid.getTargetYDegrees());
//                  // etc.
//               }
//            } else {
//               telemetry.addLine("No fiducial results list or empty");
//            }
            telemetry.addData("Result", limelight.getLatestResult());
            telemetry.addData("Target Valid", result.isValid());
            telemetry.addData("tx (Horizontal Offset)", result.getTx());
            telemetry.addData("ty (Vertical Offset)", result.getTy());
            telemetry.addData("ta (Target Area)", result.getTa());
//            telemetry.addData("ts (Skew)", result.getTs());
            telemetry.addData("txNC (Non-Clipped X)", result.getTxNC());
            telemetry.addData("tyNC (Non-Clipped Y)", result.getTyNC());
            telemetry.addData("status", limelight.getStatus());
            telemetry.addData("Bot Pose",result.getBotpose());
//            telemetry.addData("Capture Timestamp", result.getCaptureTimestamp());
         } else {
            telemetry.addLine("No valid target detected.");
         }

         telemetry.update();
         sleep(100); // refresh every 100ms
      }
   }
}
