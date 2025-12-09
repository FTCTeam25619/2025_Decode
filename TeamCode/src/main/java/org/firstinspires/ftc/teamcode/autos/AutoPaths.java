package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class AutoPaths {
    public PathChain wallToFarZonePrescore;
    public PathChain farZonePrescoreToFarIntake;
    public PathChain farIntakeToFarZoneScore;
    public PathChain park;

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
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110), 0.7)
                .setVelocityConstraint(25.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .build();

        farZonePrescoreToFarIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.742, 14.543),
                                new Pose(65.086, 36.954),
                                new Pose(24.318, 36.954)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(105), Math.toRadians(355), 0.7)
                .setVelocityConstraint(25.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .build();

        farIntakeToFarZoneScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(24.318, 36.954),
                                new Pose(56.742, 14.543)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(355), Math.toRadians(110), 0.7)
                .setVelocityConstraint(25.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .setBrakingStart(2.0)
                .build();

        park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.742, 14.543),
                                new Pose(56.742, 27.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(110), 0.7)
                .setVelocityConstraint(25.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .build();
    }
}
