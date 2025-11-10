package org.firstinspires.ftc.teamcode.pedroPathing.paths;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Util.BuildPath;

public class BlueAllianceSidePaths {
   //A_I, A_II, A_II names are geted from Actual Game Manual Page no 79

   //Ready to Collect Poses
   public static Pose A_I_CollectPose = new Pose(45, 35, Math.toRadians(180));
   public static Pose A_II_CollectPose = new Pose(45, 60, Math.toRadians(180));
   public static Pose A_III_CollectPose = new Pose(45, 83, Math.toRadians(180));

   //Collected Poses
   public static Pose A_I_CollectedPose = new Pose(13, 35, Math.toRadians(180));
   public static Pose A_II_CollectedPose = new Pose(15, 60, Math.toRadians(180));
   public static Pose A_III_CollectedPose = new Pose(19, 83, Math.toRadians(180));

   //Shooting Poses
   public static Pose A_III_ShootingPose = new Pose(45, 98, Math.toRadians(137));
   public static Pose A_I_II_ShootingPose = new Pose(52, 80, Math.toRadians(137));

   //Collect to Collected
   private static Path Collect_A_I = BuildPath.getPath(A_I_CollectPose, A_I_CollectedPose);
   private static Path Collect_A_II = BuildPath.getPath(A_II_CollectPose, A_II_CollectedPose);
   private static Path Collect_A_III = BuildPath.getPath(A_III_CollectPose, A_III_CollectedPose);

   //Collect To Shoot
   private static Path Collect_To_Shoot_Direct_A_III = BuildPath.getPath(A_III_CollectedPose, A_III_ShootingPose);
   private static Path Collect_To_Shoot_Direct_A_II = BuildPath.getPath(A_II_CollectedPose, A_I_II_ShootingPose);
   private static Path Collect_To_Shoot_Direct_A_I = BuildPath.getPath(A_I_CollectedPose, A_I_II_ShootingPose);
   private static Path Collect_To_Shoot_Curve_A_I = BuildPath.getPath(A_I_CollectedPose, new Pose(58, 40), A_I_II_ShootingPose);
   //Curve path make a curve to go for shooting which doesn't disturb the A_II Artifacts

   public static Path[] Collect_And_Shoot_A_I = {Collect_A_I, Collect_To_Shoot_Direct_A_I};
   public static Path[] Collect_And_Shoot_Curve_A_I = {Collect_A_I, Collect_To_Shoot_Curve_A_I};
   public static Path[] Collect_And_Shoot_A_II = {Collect_A_II, Collect_To_Shoot_Direct_A_II};
   public static Path[] Collect_And_Shoot_A_III = {Collect_A_III, Collect_To_Shoot_Direct_A_III};
}
