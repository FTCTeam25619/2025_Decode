package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Configurable
public class BlueFarDDConstants {
    public Pose startPose = new Pose(56.000, 8.000, Math.toRadians(90.0));
    public Pose startShootCP = new Pose(56.265, 12.397);
    public Pose shootPose = new Pose(56.742, 14.543, Math.toRadians(109.5));
    public Pose midIntakeCP = new Pose(65.324, 72.238);
    public Pose midIntakePose = new Pose(17.9, 56.24, Math.toRadians(360.0));
    public Pose openGatePose = new Pose(17.927, 56.242, Math.toRadians(90));
    public Pose openGateCP = new Pose(29.283, 63.959);
    public Pose nearIntakePose = new Pose(18.318, 36.454, Math.toRadians(360.0));
    public Pose nearIntakeCP = new Pose(65.086, 36.454);
    public Pose cornerIntakePose = new Pose(8.000, 9.000, Math.toRadians(360.0));
    public Pose cornerIntakeCP = new Pose(33.171, 18.595);
    public Pose cornerShootCP = new Pose(46.444, 25.569);
    public Pose parkPose = new Pose(56.742, 27.000, Math.toRadians(90.0));


    public PathChain preloadScore;
    public PathChain intakeMid;
    public PathChain openGate;
    public PathChain scoreMid;
    public PathChain intakeNear;
    public PathChain scoreNear;
    public PathChain intakeCorner;
    public PathChain scoreCorner;
    public PathChain park;

    public BlueFarDDConstants(Follower follower) {
        buildPaths(follower);
    }

    private void buildPaths(Follower follower) {
        preloadScore = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                startPose,
                                startShootCP,
                                shootPose
                        )
                )
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        shootPose.getHeading(),
                        0.5)
                .build();

        intakeMid = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootPose,
                                midIntakeCP,
                                midIntakePose
                        )
                )
                .setLinearHeadingInterpolation(
                        shootPose.getHeading(),
                        midIntakePose.getHeading(),
                        0.5)
                .build();

        openGate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                midIntakePose,
                                openGateCP,
                                openGatePose
                        )
                ).setLinearHeadingInterpolation(
                        midIntakePose.getHeading(),
                        openGatePose.getHeading(),
                        0.5)
                .build();

        scoreMid = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                openGatePose,
                                midIntakeCP,
                                shootPose
                        )
                )
                .setLinearHeadingInterpolation(
                        openGatePose.getHeading(),
                        shootPose.getHeading(),
                        0.5)
                .build();

        intakeNear = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootPose,
                                nearIntakeCP,
                                nearIntakePose
                        )
                )
                .setLinearHeadingInterpolation(
                        shootPose.getHeading(),
                        nearIntakePose.getHeading(),
                        0.5)
                .build();

        scoreNear = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                nearIntakePose,
                                nearIntakeCP,
                                shootPose
                        )
                )
                .setLinearHeadingInterpolation(
                        nearIntakePose.getHeading(),
                        shootPose.getHeading(),
                        0.5)
                .build();


        intakeCorner = follower.pathBuilder().addPath(
                        new BezierCurve(
                                shootPose,
                                cornerIntakeCP,
                                cornerIntakePose
                        )
                )
                .setLinearHeadingInterpolation(
                        shootPose.getHeading(),
                        cornerIntakePose.getHeading(),
                        0.5)
                .build();

        scoreCorner = follower.pathBuilder().addPath(
                        new BezierCurve(
                                cornerIntakePose,
                                cornerShootCP,
                                shootPose
                        )
                )
                .setLinearHeadingInterpolation(
                        cornerIntakePose.getHeading(),
                        shootPose.getHeading(),
                        0.5)
                .build();

        park = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootPose,
                                parkPose
                        )
                )
                .setLinearHeadingInterpolation(
                        shootPose.getHeading(),
                        parkPose.getHeading(),
                        0.5)
                .build();

    }
}
