package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "MegaTag")
public class MegaTag extends LinearOpMode {

    private Limelight3A Limelight;
    private IMU Imu;

    @Override
    public void runOpMode() throws InterruptedException {

        Limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Limelight.pipelineSwitch(6);

        Imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN);
        Imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        Limelight.setPollRateHz(50);
        telemetry.getMsTransmissionInterval();
        Limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            YawPitchRollAngles orientation = Imu.getRobotYawPitchRollAngles();
            double robotYaw = orientation.getYaw();  // <- this is your yaw angle
            Limelight.updateRobotOrientation(robotYaw);

            LLResult result = Limelight.getLatestResult();

            if (result != null && result.isValid()) {
                Pose3D botpose_mt2 = result.getBotpose_MT2();
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                telemetry.addData("Heading", robotYaw);
                telemetry.addData("Target X", result.getTx());
                telemetry.addData("Target Y", result.getTy());
                telemetry.addData("Target Area", result.getTa());
                telemetry.update();
            }
        }
    }
}

