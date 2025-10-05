package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.followerConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.localizerConstants;


import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



import org.firstinspires.ftc.teamcode.pedroPathing.paths.BlueAllianceCorner;


@Autonomous(name = "BlueAllianceCorner")
public class AutoBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

        Follower follower = new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();


        follower.setPose(BlueAllianceCorner.start);

        waitForStart();

        if (opModeIsActive()) {
            follower.followPath(BlueAllianceCorner.launch1());
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
            }
            sleep(2000);
            // Run Collection
            follower.followPath(BlueAllianceCorner.Collection());
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
            }
            sleep(2000);
            //Comeback to launch
            follower.followPath(BlueAllianceCorner.Launch2());
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
            }
        }

    }
}
