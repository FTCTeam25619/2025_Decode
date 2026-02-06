package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Configurable
public class RedFarFromGoalConstants {


    public Pose redFarWallStart = new Pose(88.000, 8.000);
    public Pose redFarWallToShootCP = new Pose(87.735, 12.397);
    public Pose redFarWallShoot = new Pose(87.258, 14.543);
    public Pose redFarWallShootToFarIntakeCP = new Pose(78.914, 36.454);
    public Pose redFarWallFarIntake = new Pose(132, 36.454);
    public Pose redFarWallShootToMidIntakeCP = new Pose(78.676, 72.238);
    public Pose redFarWallMidIntake = new Pose(132, 55.192);
    public Pose redFarWallIntakeLoadingStationCP = new Pose(93,25);
    public Pose redFarWallIntakeLoadingStation = new Pose(132, 9.00);
    public Pose redFarWallParkPose = new Pose(114.00,46.00);
    public double redFarWallStartHeading = 90;
    public double redFarWallFirstShotHeading = 71;
    public double redFarWallShootHeading = 71;
    public double redFarWallRowIntakeHeading = 180.0;
    public double redFarWallMidIntakeHeading = 180.0;

    public double redLoadingZoneIntakeHeading = 180.0;


    public PathChain redWallToFarZonePrescore;
    public PathChain redFarZonePrescoreToFarIntake;
    public PathChain redFarWallIntakeToFarScore;
    public PathChain redFarZoneScoreToMidIntake;
    public PathChain redMidWallIntakeToFarScore;
    public PathChain redFarWallToIntakeLoadingStation;
    public PathChain redFarWallPark;


    public RedFarFromGoalConstants(Follower follower) {
        buildPaths(follower);
    }

    private void buildPaths(Follower follower) {
        redWallToFarZonePrescore = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                redFarWallStart,
                                redFarWallToShootCP,
                                redFarWallShoot
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(redFarWallStartHeading),
                        Math.toRadians(redFarWallFirstShotHeading),
                        0.5)
                .build();

        redFarZonePrescoreToFarIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                redFarWallShoot,
                                redFarWallShootToFarIntakeCP,
                                redFarWallFarIntake
                        )
                )
                .setTranslationalConstraint(4.0)

                .setLinearHeadingInterpolation(
                        Math.toRadians(redFarWallShootHeading),
                        Math.toRadians(redFarWallRowIntakeHeading),
                        .5)
                .build();

        redFarWallIntakeToFarScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                redFarWallFarIntake,
                                redFarWallShoot
                        )

                )
                .setTranslationalConstraint(3.0)

                .setLinearHeadingInterpolation(
                        Math.toRadians(redFarWallRowIntakeHeading),
                        Math.toRadians(redFarWallShootHeading),
                        .5)
                .build();

        redFarZoneScoreToMidIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                redFarWallShoot,
                                redFarWallShootToMidIntakeCP,
                                redFarWallMidIntake
                        )
                )
                .setTranslationalConstraint(4.0)

                .setLinearHeadingInterpolation(
                        Math.toRadians(redFarWallShootHeading),
                        Math.toRadians(redFarWallMidIntakeHeading),
                        .5)
                .build();

        redMidWallIntakeToFarScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                redFarWallMidIntake,
                                redFarWallShoot
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(redFarWallRowIntakeHeading),
                        Math.toRadians(redFarWallShootHeading),
                        .5)
                .build();

        redFarWallToIntakeLoadingStation = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                redFarWallShoot,
                                redFarWallIntakeLoadingStationCP,
                                redFarWallIntakeLoadingStation
                        )
                )
                .setTranslationalConstraint(4.0)
                .setLinearHeadingInterpolation(
                        Math.toRadians(redFarWallShootHeading),
                        Math.toRadians(redLoadingZoneIntakeHeading),
                        0.5)
                .build();
        redFarWallPark = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                redFarWallIntakeLoadingStation,
                                redFarWallParkPose
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(redLoadingZoneIntakeHeading),
                        Math.toRadians(90.0),
                        0.5)
                .build();
    }
}
