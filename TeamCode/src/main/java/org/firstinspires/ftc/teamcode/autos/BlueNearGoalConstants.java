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
    public Pose blueNearWallToShootCP = new Pose(28.624, 109.431);
    public Pose blueNearWallShoot = new Pose(48.606, 103.009);
    public Pose blueNearWallShootToNearIntakeCP = new Pose(63.193, 63.615);
    public Pose blueNearWallNearIntake = new Pose(18.5, 86.532);
    public Pose blueNearWallShootToMidIntakeCP = new Pose(64.294, 38.514);
    public Pose blueNearWallMidIntake = new Pose(20.0, 56.5);
    public Pose blueNearWallPark = new Pose(54.606, 114.275);
    public double blueNearWallStartHeading = 90.0;
    public double blueNearWallShootHeading = 147.0;
    public double blueNearWallRowIntakeHeading = 0.0;
    public double blueNearWallMidRowIntakeHeading = 0.0;


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
                        0.5)
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
                .build();

        blueNearZoneScoreToMidIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                blueNearWallShoot,
                                blueNearWallShootToMidIntakeCP,
                                blueNearWallMidIntake
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueNearWallShootHeading),
                        Math.toRadians(blueNearWallMidRowIntakeHeading),
                        0.4)
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
                .build();
    }
}
