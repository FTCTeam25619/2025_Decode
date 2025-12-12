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
    public Pose redFarWallFarIntake = new Pose(128.682, 33.454);
    public Pose redFarWallShootToMidIntakeCP = new Pose(78.676, 72.238);
    public Pose redFarWallMidIntake = new Pose(115.0, 55.192);
    public Pose redFarWallPark = new Pose(87.258, 35.000);
    public double redFarWallStartHeading = 90.0;
    public double redFarWallFirstShotHeading = 68.0;
    public double redFarWallShootHeading = 67.5;
    public double redFarWallRowIntakeHeading = 175.0;
    public double redFarWallMidIntakeHeading = 190.0;

    public PathChain redWallToFarZonePrescore;
    public PathChain redFarZonePrescoreToFarIntake;
    public PathChain redFarWallIntakeToFarScore;
    public PathChain redFarZoneScoreToMidIntake;
    public PathChain redMidWallIntakeToFarScore;
    public PathChain redFarWallToPark;

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
                .setVelocityConstraint(25.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(1.0))
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
                .setLinearHeadingInterpolation(
                        Math.toRadians(redFarWallShootHeading),
                        Math.toRadians(redFarWallRowIntakeHeading),
                        0.7)
                .setVelocityConstraint(25.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .build();

        redFarWallIntakeToFarScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                redFarWallFarIntake,
                                redFarWallShoot
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(redFarWallRowIntakeHeading),
                        Math.toRadians(redFarWallShootHeading),
                        0.5)
                .setVelocityConstraint(25.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(1.0))
                .setBrakingStart(2.0)
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
                .setLinearHeadingInterpolation(
                        Math.toRadians(redFarWallShootHeading),
                        Math.toRadians(redFarWallMidIntakeHeading),
                        0.5)
                .setVelocityConstraint(25.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
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
                        0.5)
                .setVelocityConstraint(25.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(1.0))
                .setBrakingStart(2.0)
                .build();

        redFarWallToPark = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                redFarWallShoot,
                                redFarWallPark
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(redFarWallShootHeading),
                        Math.toRadians(redFarWallShootHeading),
                        0.7)
                .setVelocityConstraint(25.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .build();
    }
}
