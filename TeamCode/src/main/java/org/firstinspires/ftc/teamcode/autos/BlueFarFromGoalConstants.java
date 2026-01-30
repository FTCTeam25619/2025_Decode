package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Configurable
public class BlueFarFromGoalConstants {
    public Pose blueFarWallStart = new Pose(56.000, 8.000);
    public Pose blueFarWallToShootCP = new Pose(56.265, 12.397);
    public Pose blueFarWallShoot = new Pose(56.742, 14.543);
    public Pose blueFarWallShootToFarIntakeCP = new Pose(65.086, 36.454);
    public Pose blueFarWallFarIntake = new Pose(21.0, 36.454);
    public Pose blueFarWallShootToMidIntakeCP = new Pose(65.324, 72.238);
    public Pose blueFarWallMidIntake = new Pose(18.0, 55.192);
    public Pose blueFarWallParkCP = new Pose(51,25);
    public Pose blueFarWallPark = new Pose(15.00, 10.00);
    public double blueFarWallStartHeading = 90.0;
    public double blueFarWallFirstShotHeading = 113.0;
    public double blueFarWallShootHeading = 113.0;
    public double blueFarWallRowIntakeHeading = 0.0;
    public double blueFarWallMidIntakeHeading = 0.0;

    public PathChain blueWallToFarZonePrescore;
    public PathChain blueFarZonePrescoreToFarIntake;
    public PathChain blueFarWallIntakeToFarScore;
    public PathChain blueFarZoneScoreToMidIntake;
    public PathChain blueMidWallIntakeToFarScore;
    public PathChain blueFarWallToPark;

    public BlueFarFromGoalConstants(Follower follower) {
        buildPaths(follower);
    }

    private void buildPaths(Follower follower) {
        blueWallToFarZonePrescore = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                blueFarWallStart,
                                blueFarWallToShootCP,
                                blueFarWallShoot
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueFarWallStartHeading),
                        Math.toRadians(blueFarWallFirstShotHeading),
                        0.5)
                .build();

        blueFarZonePrescoreToFarIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                blueFarWallShoot,
                                blueFarWallShootToFarIntakeCP,
                                blueFarWallFarIntake
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueFarWallShootHeading),
                        Math.toRadians(blueFarWallRowIntakeHeading),
                .5)
                .build();

        blueFarWallIntakeToFarScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                blueFarWallFarIntake,
                                blueFarWallShoot
                        )

                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueFarWallRowIntakeHeading),
                        Math.toRadians(blueFarWallShootHeading),
                .5)
                .build();

        blueFarZoneScoreToMidIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                blueFarWallShoot,
                                blueFarWallShootToMidIntakeCP,
                                blueFarWallMidIntake
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueFarWallShootHeading),
                        Math.toRadians(blueFarWallMidIntakeHeading),
                .5)
                .build();

        blueMidWallIntakeToFarScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                blueFarWallMidIntake,
                                blueFarWallShoot
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueFarWallRowIntakeHeading),
                        Math.toRadians(blueFarWallShootHeading),
                .5)
                .build();

        blueFarWallToPark = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                blueFarWallShoot,
                                blueFarWallParkCP,
                                blueFarWallPark
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueFarWallShootHeading),
                        Math.toRadians(0.0),
                0.5)
                .build();
    }
}
