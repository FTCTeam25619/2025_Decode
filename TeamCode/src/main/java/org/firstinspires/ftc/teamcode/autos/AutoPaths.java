package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Configurable
public class AutoPaths {
    public Pose blueFarWallStart = new Pose(56.000, 8.000);
    public Pose blueFarWallShoot = new Pose(56.742, 14.543);
    public Pose blueFarWallRowIntake = new Pose(24.318, 36.954);
    public double farWallStartHeading = 90.0;
    public double blueFarWallShootHeading = 110.0;
    public double blueFarWallRowIntakeHeading = 355.0;

    public PathChain blueWallToFarZonePrescore;
    public PathChain blueFarZonePrescoreToFarIntake;
    public PathChain blueFarIntakeToFarZoneScore;
    public PathChain bluePark;

    public AutoPaths(Follower follower) {
        blueWallToFarZonePrescore = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                blueFarWallStart,
                                new Pose(56.265, 12.397),
                                blueFarWallShoot
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(farWallStartHeading),
                        Math.toRadians(blueFarWallShootHeading),
                        0.7)
                .setVelocityConstraint(25.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .build();

        blueFarZonePrescoreToFarIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                blueFarWallShoot,
                                new Pose(65.086, 36.954),
                                blueFarWallRowIntake
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueFarWallShootHeading),
                        Math.toRadians(blueFarWallRowIntakeHeading),
                        0.7)
                .setVelocityConstraint(25.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .build();

        blueFarIntakeToFarZoneScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                blueFarWallRowIntake,
                                blueFarWallShoot
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueFarWallRowIntakeHeading),
                        Math.toRadians(blueFarWallShootHeading),
                        0.7)
                .setVelocityConstraint(25.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .setBrakingStart(2.0)
                .build();

        bluePark = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                blueFarWallShoot,
                                new Pose(56.742, 27.000)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueFarWallShootHeading),
                        Math.toRadians(blueFarWallShootHeading),
                        0.7)
                .setVelocityConstraint(25.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .build();
    }
}
