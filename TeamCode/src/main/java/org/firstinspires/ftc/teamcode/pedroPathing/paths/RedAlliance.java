package org.firstinspires.ftc.teamcode.pedroPathing.paths;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.pathConstraints;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.paths.PathBuilder;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

public class RedAlliance {
    public static PathBuilder builder = new PathBuilder(follower);

    public static PathChain line1 = builder
            .addPath(new BezierLine(new Pose(60.000, 8.000), new Pose(60.000, 39.000)))
            .setConstantHeadingInterpolation(Math.toRadians(90))
            .build();

    public static PathChain line2 = builder
            .addPath(new BezierLine(new Pose(60.000, 47.000), new Pose(60.000, 47.000)))
            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(50))
            .build();

    public static PathChain line3 = builder
            .addPath(
                    new BezierLine(new Pose(60.000, 47.000), new Pose(106.000, 105.000))
            )
            .setTangentHeadingInterpolation()
            .setReversed()
            .build();

}
