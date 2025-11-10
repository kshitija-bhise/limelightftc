package org.firstinspires.ftc.teamcode.pedroPathing.paths;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.BuildPath;
import org.firstinspires.ftc.teamcode.Vision.DistanceEstimator;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Configurable
@TeleOp(name = "Blue Alliance Motif Select")
public class BlueAllianceMotifSelect extends LinearOpMode {
   public static int motifID = 0;
   int state = -1;
   Pose startPose = new Pose(55, 135, Math.toRadians(90));
   Follower follower;
   ArrayList<Path> paths = new ArrayList<>();
   DistanceEstimator distanceEstimator;

   @Override
   public void runOpMode() throws InterruptedException {
      follower = Constants.createFollower(hardwareMap);
      follower.setMaxPower(0.5);
      distanceEstimator = new DistanceEstimator(hardwareMap.get(Limelight3A.class, "limelight"), 19, 11, 29.5);
      Pose middlePose = new Pose(43.235, 110.021);
      Path startPath1 = new Path(new BezierCurve(startPose, middlePose));
      startPath1.setConstantHeadingInterpolation(Math.toRadians(90));
      Path startPath2 = new Path(new BezierCurve(middlePose, BlueAllianceSidePaths.A_III_ShootingPose));
      startPath2  .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(137));

      paths.add(startPath1);
      paths.add(startPath2);

      follower.setStartingPose(startPose);
      setPaths(motifID);
      waitForStart();
      while (opModeIsActive()) {
         if (!follower.isBusy()) {
//            switch (state) {
//               case 0:
//                  autoShoot();
//                  break;
//               case 1:
//                  intake.startIntake();
//                  break;
//               case 2:
//                  intake.slowIntake();
//                  break;
//               case 3:
//                  autoShoot();
//                  break;
//               case 4:
//                  intake.startIntake();
//                  break;
//               case 5:
//                  intake.slowIntake();
//                  break;
//               case 6:
//                  autoShoot();
//                  break;
//               case 7:
//                  intake.startIntake();
//                  break;
//               case 8:
//                  intake.slowIntake();
//                  break;
//               case 9:
//                  autoShoot();
//                  break;
//            }
            state++;   //increment of the state
            if (state == 0 || state == 4 || state == 7 || state == 1) {
               follower.followPath(paths.get(state), false);  //runs in a continuous manner
               follower.setMaxPower(1);
            } else {
               if (!(state == 2 || state == 5 || state == 8 || state == 3)) {
                  sleep(500);
               }
               follower.setMaxPower(1);
               follower.followPath(paths.get(state));
            }

         } else {
            switch (state) {
               case 0:
                  int id = distanceEstimator.getID();
                  if (id != 0 && motifID == 0) {
                     motifID = distanceEstimator.getID();
                     setPaths(motifID);
                  }

//                  shooter.startShooter();
                  break;
               case 1:
                  break;
               case 2:
                  break;
               case 3:
//                  shooter.startShooter();
                  break;
               case 4:
                  break;
               case 5:
                  break;
               case 6:
//                  shooter.startShooter();
                  break;
               case 7:
                  break;
               case 8:
                  break;
               case 9:
//                  shooter.startShooter();
                  break;
            }
         }
         follower.update();
         telemetry.addData("BotPose: ", follower.getPose());
         telemetry.addData("MotifID", motifID);
         telemetry.update();
      }

   }

   private void setPaths(int motifID) {
      //21 - GPP, 22 - PGP, 23 - PPG
      if (motifID == 21) {
         paths.add(BuildPath.getPath(BlueAllianceSidePaths.A_III_ShootingPose, BlueAllianceSidePaths.A_III_CollectPose));
         paths.addAll(Arrays.asList(BlueAllianceSidePaths.Collect_And_Shoot_A_III));

         paths.add(BuildPath.getPath(BlueAllianceSidePaths.A_III_ShootingPose, BlueAllianceSidePaths.A_II_CollectPose));
         paths.addAll(Arrays.asList(BlueAllianceSidePaths.Collect_And_Shoot_A_II));

         paths.add(BuildPath.getPath(BlueAllianceSidePaths.A_I_II_ShootingPose, BlueAllianceSidePaths.A_I_CollectPose));
         paths.addAll(Arrays.asList(BlueAllianceSidePaths.Collect_And_Shoot_A_I));
      } else if (motifID == 22) {
         paths.add(BuildPath.getPath(BlueAllianceSidePaths.A_III_ShootingPose, BlueAllianceSidePaths.A_II_CollectPose));
         paths.addAll(Arrays.asList(BlueAllianceSidePaths.Collect_And_Shoot_A_II));

         paths.add(BuildPath.getPath(BlueAllianceSidePaths.A_I_II_ShootingPose, BlueAllianceSidePaths.A_III_CollectPose));
         paths.addAll(Arrays.asList(BlueAllianceSidePaths.Collect_And_Shoot_A_III));

         paths.add(BuildPath.getPath(BlueAllianceSidePaths.A_III_ShootingPose, BlueAllianceSidePaths.A_I_CollectPose));
         paths.addAll(Arrays.asList(BlueAllianceSidePaths.Collect_And_Shoot_A_I));
      } else if (motifID == 23) {
         paths.add(BuildPath.getPath(BlueAllianceSidePaths.A_III_ShootingPose, BlueAllianceSidePaths.A_I_CollectPose));
         paths.addAll(Arrays.asList(BlueAllianceSidePaths.Collect_And_Shoot_Curve_A_I));
         paths.add(BuildPath.getPath(BlueAllianceSidePaths.A_I_II_ShootingPose, BlueAllianceSidePaths.A_III_CollectPose));
         paths.addAll(Arrays.asList(BlueAllianceSidePaths.Collect_And_Shoot_A_III));
         paths.add(BuildPath.getPath(BlueAllianceSidePaths.A_III_ShootingPose, BlueAllianceSidePaths.A_II_CollectPose));
         paths.addAll(Arrays.asList(BlueAllianceSidePaths.Collect_And_Shoot_A_II));
      }
   }
}
