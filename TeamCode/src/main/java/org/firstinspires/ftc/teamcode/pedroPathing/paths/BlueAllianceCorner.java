package org.firstinspires.ftc.teamcode.pedroPathing.paths;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.paths.BlueAlliance.builder;

import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;

import org.firstinspires.ftc.teamcode.Hardware.Claw;
import org.opencv.core.Mat;
import org.opencv.core.Point;

public class BlueAllianceCorner {
        public static PathBuilder builder = new PathBuilder(follower);
        public static Pose start = new Pose(24.6, 122.6, Math.toRadians(-45));
        public static Pose score1 = new Pose(48, 99.6);
        public static Pose Collect1 = new Pose(33.44, 84, Math.toRadians(-45));


        public static PathChain launch1(){
                return new PathBuilder(follower)
                        .addPath(
                                new BezierLine(
                                        start,
                                        score1
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(-45))
                        //.setZeroPowerAcceleration
                        .build();
        }

        public static PathChain Collection() {
                return new PathBuilder(follower)
                        .addPath(
                                new BezierCurve(
                                        score1,
                                        new Pose(56.698, 88.962),
                                        new Pose(33.450, 84.692)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-180))
                        //.setZeroPowerAcceleration
                        .build();

        }

        public static PathChain Launch2() {
                return new PathBuilder(follower)
                        .addPath(
                                new BezierCurve(
                                        new Pose(34.450, 84.692),
                                        new Pose(56.936, 89.199),
                                        new Pose(47.921, 99.638)
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .build();

        }

        public static PathChain Collect2(){
                return new PathBuilder(follower)
                        .addPath(
                                new BezierCurve(
                                        new Pose(47.921, 100.112),
                                        new Pose(76.863, 70.695),
                                        new Pose(37.245, 60.494)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-180))
                        .build();
        }

        public static  PathChain Launch3(){
                return new PathBuilder(follower)
                        .addPath(
                                new BezierCurve(
                                        new Pose(37.245, 60.494),
                                        new Pose(76.389, 70.695),
                                        new Pose(48.158, 99.400)
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .build();
        }
}






