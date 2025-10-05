package org.firstinspires.ftc.teamcode.pedroPathing.paths;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;

public class TESTPATH {
    public static PathBuilder builder = new PathBuilder(follower);

    public static PathChain test(){
        return new PathBuilder(follower)
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Pose(95.000, 105.000),
                                new Pose(95.000, 105.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(45))
                .build();
    }
}
