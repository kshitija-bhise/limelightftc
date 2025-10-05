package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Autonomous(name ="Yaw Test")
public class YawTest extends LinearOpMode {

    public Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(6);

        limelight.start();

        waitForStart();

        while (opModeIsActive()){

            LLStatus status = limelight.getStatus();

            LLResult result = limelight.getLatestResult();


            if (result.isValid()){
                Pose3D botpose = result.getBotpose();

                double Yaw = botpose.getOrientation().getYaw();

                double final_yaw = Yaw + 125;

                telemetry.addData("Final Yaw", final_yaw);

                telemetry.update();
            } else {
                telemetry.addData("Limelight", "No data available");
                telemetry.update();
            }
        }


    }
}
