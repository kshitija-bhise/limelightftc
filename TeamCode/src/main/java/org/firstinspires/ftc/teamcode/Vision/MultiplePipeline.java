package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

@Autonomous
public class MultiplePipeline extends LinearOpMode {
    private Limelight3A limelight;
    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class,"limelight");

        limelight.pipelineSwitch(1);
        limelight.start();

        telemetry.setMsTransmissionInterval(11);

        waitForStart();

        while (opModeIsActive()){
            LLResult result = limelight.getLatestResult();

            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
               double TagID = fr.getFiducialId();

               if (TagID == 21){
                   telemetry.addData("TagID","21");
               } else if (TagID == 22) {
                   telemetry.addData("TagID","22");
               } else if (TagID == 23) {
                   telemetry.addData("TagID","23");
               }

               telemetry.update();
            }


        }



    }
}
