package org.firstinspires.ftc.teamcode.pedroPathing.paths;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class BlueAlliance {
        public static PathBuilder builder = new PathBuilder(follower);

        public static PathChain paths = builder
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Pose(83.743, 4.745),
                                new Pose(92.995, 64.527),
                                new Pose(52.000, 98.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(140))
                .build();
    }
