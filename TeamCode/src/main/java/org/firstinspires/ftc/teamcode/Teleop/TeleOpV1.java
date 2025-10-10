package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.followerConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.ActiveIntake;
import org.firstinspires.ftc.teamcode.Hardware.ServoAngle;
import org.firstinspires.ftc.teamcode.Hardware.ServoTurn;
import org.firstinspires.ftc.teamcode.Hardware.Shooter;

@TeleOp(name = "DecodeV1")
public class TeleOpV1 extends LinearOpMode {
    private Follower follower;
    private Shooter shooter;
    private ActiveIntake Intake;
    private ServoAngle servoAngle;
    private ServoTurn servoTurn;

    @Override
    public void runOpMode() throws InterruptedException {
        Intake = new ActiveIntake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        servoAngle = new ServoAngle(hardwareMap);
        servoTurn = new ServoTurn(hardwareMap);

      //  PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
        follower = new FollowerBuilder(followerConstants, hardwareMap)
             //   .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();

        follower.startTeleopDrive();

        waitForStart();

        while (opModeIsActive()) {

            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();


            if(gamepad1.a){
                Intake.startIntake();
            }
            if (gamepad1.b) servoAngle.setServoAdjust(0.1);

        }

    }
}
