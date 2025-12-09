package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Configurable
public class BlueNearGoalConstants {

    public Pose blueNearWallStart = new Pose(15.413, 114.055);
    public Pose blueNearWallToShootCP = new Pose(36.110, 114.275);
    public Pose blueNearWallShoot = new Pose(54.606, 107.009);
    public Pose blueNearWallShootToNearIntakeCP = new Pose(63.853, 75.523);
    public Pose blueNearWallNearIntake = new Pose(22.899, 87.633);
    public Pose blueNearWallShootToMidIntakeCP = new Pose(65.174, 56.367);
    public Pose blueNearWallMidIntake = new Pose(22.459, 63.193);
    public Pose blueNearWallPark = new Pose(54.606, 114.275);
    public double blueNearWallStartHeading = 90.0;
    public double blueNearWallShootHeading = 135.0;
    public double blueNearWallRowIntakeHeading = 10.0;

    public PathChain blueWallToNearZonePrescore;
    public PathChain blueNearZonePrescoreToNearIntake;
    public PathChain blueNearWallIntakeToNearScore;
    public PathChain blueNearZoneScoreToMidIntake;
    public PathChain blueMidWallIntakeToNearScore;
    public PathChain blueNearWallToPark;

    public BlueNearGoalConstants(Follower follower) {
        buildPaths(follower);
    }

    private void buildPaths(Follower follower) {
        blueWallToNearZonePrescore = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                blueNearWallStart,
                                blueNearWallToShootCP,
                                blueNearWallShoot
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueNearWallStartHeading),
                        Math.toRadians(blueNearWallShootHeading),
                        0.7)
                .setVelocityConstraint(25.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .build();

        blueNearZonePrescoreToNearIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                blueNearWallShoot,
                                blueNearWallShootToNearIntakeCP,
                                blueNearWallNearIntake
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueNearWallShootHeading),
                        Math.toRadians(blueNearWallRowIntakeHeading),
                        0.7)
                .setVelocityConstraint(25.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .build();

        blueNearWallIntakeToNearScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                blueNearWallNearIntake,
                                blueNearWallShoot
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueNearWallRowIntakeHeading),
                        Math.toRadians(blueNearWallShootHeading),
                        0.7)
                .setVelocityConstraint(25.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .setBrakingStart(2.0)
                .build();

        blueNearZoneScoreToMidIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                blueNearWallShoot,
                                blueNearWallShootToMidIntakeCP,
                                new Pose(22.899, 60.550)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueNearWallShootHeading),
                        Math.toRadians(blueNearWallRowIntakeHeading),
                        0.7)
                .setVelocityConstraint(25.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .build();

        blueMidWallIntakeToNearScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                blueNearWallMidIntake,
                                blueNearWallShoot
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueNearWallRowIntakeHeading),
                        Math.toRadians(blueNearWallShootHeading),
                        0.7)
                .setVelocityConstraint(25.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .setBrakingStart(2.0)
                .build();

        blueNearWallToPark = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                blueNearWallShoot,
                                blueNearWallPark
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueNearWallShootHeading),
                        Math.toRadians(blueNearWallShootHeading),
                        0.7)
                .setVelocityConstraint(25.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .build();
    }
}
