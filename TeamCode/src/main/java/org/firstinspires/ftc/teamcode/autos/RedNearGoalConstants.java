package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Configurable
public class RedNearGoalConstants {
    public Pose redNearWallStart = new Pose(128.587, 114.055);
    public Pose redNearWallToShootCP = new Pose(115.376, 109.431);
    public Pose redNearWallShoot = new Pose(95.394, 101.009);
    public Pose redNearWallShootToNearIntakeCP = new Pose(80.807, 63.615);
    public Pose redNearWallNearIntake = new Pose(122.5, 87.0);
    public Pose redNearWallShootToMidIntakeCP = new Pose(79.706, 38.514);
    public Pose redNearWallMidIntake = new Pose(114.0, 58.193);
    public Pose redNearWallPark = new Pose(89.394, 114.275);
    public double redNearWallStartHeading = 90.0;
    public double redNearWallShootHeading = 42.0;
    public double redNearWallRowIntakeHeading = 175.0;
    public double redNearWallMidRowIntakeHeading = 190.0;

    public PathChain redWallToNearZonePrescore;
    public PathChain redNearZonePrescoreToNearIntake;
    public PathChain redNearWallIntakeToNearScore;
    public PathChain redNearZoneScoreToMidIntake;
    public PathChain redMidWallIntakeToNearScore;
    public PathChain redNearWallToPark;
    
    public RedNearGoalConstants(Follower follower) {
        buildPaths(follower);
    }

    private void buildPaths(Follower follower) {
        redWallToNearZonePrescore = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                redNearWallStart,
                                redNearWallToShootCP,
                                redNearWallShoot
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(redNearWallStartHeading),
                        Math.toRadians(redNearWallShootHeading),
                        0.7)
                .setVelocityConstraint(25.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .build();

        redNearZonePrescoreToNearIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                redNearWallShoot,
                                redNearWallShootToNearIntakeCP,
                                redNearWallNearIntake
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(redNearWallShootHeading),
                        Math.toRadians(redNearWallRowIntakeHeading),
                        0.5)
                .setVelocityConstraint(10.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .build();

        redNearWallIntakeToNearScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                redNearWallNearIntake,
                                redNearWallShoot
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(redNearWallRowIntakeHeading),
                        Math.toRadians(redNearWallShootHeading),
                        0.7)
                .setVelocityConstraint(25.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .setBrakingStart(2.0)
                .build();

        redNearZoneScoreToMidIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                redNearWallShoot,
                                redNearWallShootToMidIntakeCP,
                                redNearWallMidIntake
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(redNearWallShootHeading),
                        Math.toRadians(redNearWallMidRowIntakeHeading),
                        0.4)
                .setVelocityConstraint(20.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .build();

        redMidWallIntakeToNearScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                redNearWallMidIntake,
                                redNearWallShoot
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(redNearWallRowIntakeHeading),
                        Math.toRadians(redNearWallShootHeading),
                        0.7)
                .setVelocityConstraint(25.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .setBrakingStart(2.0)
                .build();

        redNearWallToPark = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                redNearWallShoot,
                                redNearWallPark
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(redNearWallShootHeading),
                        Math.toRadians(redNearWallShootHeading),
                        0.7)
                .setVelocityConstraint(25.0)
                .setTranslationalConstraint(0.5)
                .setHeadingConstraint(Math.toRadians(2.0))
                .build();
    }
}
