package org.firstinspires.ftc.teamcode.pedroPathing.paths;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class RedAllianceCorner {
    public static PathBuilder builder = new PathBuilder(follower);
    public static Pose start = new Pose(118, 124, Math.toRadians(-135));
    public static Pose score1 = new Pose(100.5, 106.7);
    public static Pose Collect1 = new Pose(33.44, 84, Math.toRadians(-45));


    public static PathChain launch1(){
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                start,
                                score1
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-135))
                //.setZeroPowerAcceleration
                .build();
    }

    public static PathChain Collection() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                score1,
                                new Pose(69.509, 70.932),
                                new Pose(109.601, 74.491)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(-1))
                //.setZeroPowerAcceleration
                .build();

    }
    public static PathChain Collect(){
        return new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(109.601, 74.491),
                                new Pose(126.208, 69.984)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-1))
                .build();
    }

    public static PathChain Launch2() {
        return new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(117.667, 83.269),
                                new Pose(70.695, 80.659),
                                new Pose(101.061, 106.992)
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

