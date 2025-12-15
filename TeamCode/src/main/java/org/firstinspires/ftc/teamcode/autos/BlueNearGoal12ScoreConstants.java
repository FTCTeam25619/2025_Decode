package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Configurable
public class BlueNearGoal12ScoreConstants {

    public Pose blueNearWallStart = new Pose(15.413, 114.055);
    public Pose blue12NearIntake = new Pose(23.603, 87.974);
    public Pose blue12MidIntake = new Pose(23.841, 65.325);
    public Pose blue12FarIntake = new Pose(24.079, 40.053);
    public Pose blue12NearShoot = new Pose(46.252, 102.517);
    public Pose blue12NearWallPark = new Pose(46.013, 57.695);
    public Pose blue12NearShootToFarIntakeCP = new Pose(18.596, 77.007);
    public Pose blue12NearShootToMidIntakeCP = new Pose(19.550, 88.689);

    public double blueNearWallStartHeading = 90.0;
    public double blue12NearShootHeading = 135.0;
    public double blue12NearIntakeHeading = 120.0;
    public double blue12MidIntakeHeading = 95.0;
    public double blue12FarIntakeHeading = 100.0;

    public PathChain blue12NearStartToNearIntake;
    public PathChain blue12NearIntakeToNearShoot;
    public PathChain blue12NearShootToMidIntake;
    public PathChain blue12MidIntakeToNearShoot;
    public PathChain blue12NearShootToFarIntake;
    public PathChain blue12FarIntakeToNearShoot;
    public PathChain blue12ShootToPark;

    public double defaultVelocityConstraint = 20.0;

    public BlueNearGoal12ScoreConstants(Follower follower) {
        buildPaths(follower);
    }

    private void buildPaths(Follower follower) {
        blue12NearStartToNearIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(blueNearWallStart, blue12NearIntake)
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueNearWallStartHeading),
                        Math.toRadians(blue12NearIntakeHeading),
                        0.7)
                .setVelocityConstraint(defaultVelocityConstraint)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .build();

        blue12NearIntakeToNearShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(blue12NearIntake, blue12NearShoot)
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blue12NearIntakeHeading),
                        Math.toRadians(blue12NearShootHeading),
                        0.7)
                .setVelocityConstraint(defaultVelocityConstraint)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .build();

        blue12NearShootToMidIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                blue12NearShoot,
                                blue12NearShootToMidIntakeCP,
                                blue12MidIntake
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blue12NearShootHeading),
                        Math.toRadians(blue12MidIntakeHeading),
                        0.5)
                .setVelocityConstraint(defaultVelocityConstraint)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .build();

        blue12MidIntakeToNearShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(blue12MidIntake, blue12NearShoot)
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blue12MidIntakeHeading),
                        Math.toRadians(blue12NearShootHeading),
                        0.7)
                .setVelocityConstraint(defaultVelocityConstraint)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .build();

        blue12NearShootToFarIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                blue12NearShoot,
                                blue12NearShootToFarIntakeCP,
                                blue12FarIntake
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blue12NearShootHeading),
                        Math.toRadians(blue12FarIntakeHeading),
                        0.6)
                .setVelocityConstraint(defaultVelocityConstraint)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .build();

        blue12FarIntakeToNearShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(blue12FarIntake, blue12NearShoot)
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blue12FarIntakeHeading),
                        Math.toRadians(blue12NearShootHeading),
                        0.7)
                .setVelocityConstraint(defaultVelocityConstraint)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .build();

        blue12ShootToPark = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(blue12NearShoot, blue12NearWallPark)
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blue12NearShootHeading),
                        Math.toRadians(blueNearWallStartHeading),
                        0.8)
                .setVelocityConstraint(defaultVelocityConstraint)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .build();
    }
}
