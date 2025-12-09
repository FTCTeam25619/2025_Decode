package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Configurable
public class RedFarFromGoalConstants {
    public Pose redFarWallStart = new Pose(72.000, 8.000);
    public Pose redFarWallShoot = new Pose(71.258, 14.543);
    public Pose redFarWallRowIntake = new Pose(103.682, 36.954);

    public double redFarWallStartHeading = 90.0;
    public double redFarWallShootHeading = 70.0;
    public double redFarWallRowIntakeHeading = 185.0;

    public PathChain redWallToFarZonePrescore;
    public PathChain redFarZonePrescoreToFarIntake;
    public PathChain redFarIntakeToFarZoneScore;
    public PathChain redPark;


    public RedFarFromGoalConstants(Follower follower) {
        buildPaths(follower);
    }

    private void buildPaths(Follower follower) {
        redWallToFarZonePrescore = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                redFarWallStart,
                                new Pose(71.735, 12.397),
                                redFarWallShoot
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(redFarWallStartHeading),
                        Math.toRadians(redFarWallShootHeading),
                        0.7)
                .setVelocityConstraint(25.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .build();

        redFarZonePrescoreToFarIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                redFarWallShoot,
                                new Pose(62.914, 36.954),
                                redFarWallRowIntake
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

        redFarIntakeToFarZoneScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                redFarWallRowIntake,
                                redFarWallShoot
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(redFarWallRowIntakeHeading),
                        Math.toRadians(redFarWallShootHeading),
                        0.7)
                .setVelocityConstraint(25.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .setBrakingStart(2.0)
                .build();

        redPark = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                redFarWallShoot,
                                new Pose(71.258, 27.000)
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
