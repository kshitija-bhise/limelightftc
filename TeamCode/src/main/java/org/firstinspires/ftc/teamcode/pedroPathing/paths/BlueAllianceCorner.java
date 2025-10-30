package org.firstinspires.ftc.teamcode.pedroPathing.paths;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;


import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import org.opencv.core.Mat;
import org.opencv.core.Point;

public class BlueAllianceCorner {
        public static PathBuilder builder = new PathBuilder(follower);
        public static Pose start = new Pose(28.5, 128, Math.toRadians(180));
        public static Pose score1 = new Pose(60,85,Math.toRadians(135));

        public static PathChain launch1(){
                return new PathBuilder(follower)
                        .addPath(
                                new BezierLine(
                                        start,
                                        score1
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                        .build();
        }

        public static PathChain Collection() {
                return new PathBuilder(follower)
                        .addPath(
                                new BezierCurve(
                                        new Pose(33.213, 114.820),
                                        new Pose(65.476, 80.659),
                                        new Pose(35.585, 83.980)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                        .build();

        }

        public static PathChain Launch2() {
                return new PathBuilder(follower)
                        .addPath(
                                new BezierCurve(
                                        new Pose(35.585, 83.980),
                                        new Pose(65.713, 80.422),
                                        new Pose(44.837, 105.094)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                        .build();

        }

        public static PathChain Collect2(){
                return new PathBuilder(follower)
                        .addPath(
                                new BezierCurve(
                                        new Pose(45.549, 105.094),
                                        new Pose(78.761, 59.783),
                                        new Pose(36.059, 60.494)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                        .build();
        }

        public static  PathChain Launch3(){
                return new PathBuilder(follower)
                        .addPath(
                                new BezierCurve(
                                        new Pose(36.059, 60.494),
                                        new Pose(78.761, 59.783),
                                        new Pose(45.311, 105.094)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                        .build();
        }

        public static PathChain Test(){
                return new PathBuilder(follower)
                        .addPath(
                                new BezierLine(
                                        new Pose(72.119, 8.303),
                                        new Pose(72.119, 69.984)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build();
        }
}






