package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class AutoPaths {
    public PathChain wallToFarZonePrescore;

    public AutoPaths(Follower follower) {
        wallToFarZonePrescore = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.000, 8.000),
                                new Pose(56.265, 12.397),
                                new Pose(56.742, 14.543)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))
                .build();
    }
}
