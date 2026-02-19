package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.lib.orchestration.ShotCycleState;

@Configurable
public class BlueFarFromGoalConstants {
    // Shared poses (used by both regular and gate paths)
    public Pose blueFarWallStart = new Pose(56.000, 8.000);
    public Pose blueFarWallToShootCP = new Pose(56.265, 12.397);
    public Pose blueFarWallShoot = new Pose(56.742, 14.543);
    public Pose blueFarWallShootToFarIntakeCP = new Pose(65.086, 35.0);
    public Pose blueFarWallFarIntake = new Pose(13.75, 35.0);
    public Pose blueFarWallShootToMidIntakeCP = new Pose(65.324, 72.238);
    public Pose blueFarWallMidIntake = new Pose(13.75, 55.192);
    public Pose blueFarWallIntakeLoadingStationCP = new Pose(51, 25);
    public Pose blueFarWallIntakeLoadingStation = new Pose(11.5, 7.75);
    public Pose blueFarWallParkPose = new Pose(30.00, 46.00);

    // Gate-specific poses

    public Pose blueGateOpenCP = new Pose(23.321, 62.764);
    public Pose blueGateOpen = new Pose(16.5, 65.000);
    public Pose blueWallIntakeGate = new Pose(15.0, 58.0);
    // Shared headings
    public double blueFarWallStartHeading = 90.0;
    public double blueFarWallFirstShotHeading = 113.5;
    public double blueFarWallShootHeading = 113.25;
    public double blueFarWallRowIntakeHeading = 0.0;
    public double blueFarWallMidIntakeHeading = 0.0;
    public double blueLoadingZoneIntakeHeading = 0.0;

    // Gate-specific headings
    public double blueGateOpenHeading = 90.0;
    public double blueGateParkHeading = 90.0;

    public PathChain blueWallToFarZonePrescore;
    public PathChain blueFarZonePrescoreToFarIntake;
    public PathChain blueFarWallIntakeToFarScore;
    public PathChain blueFarZoneScoreToMidIntake;
    public PathChain blueMidWallIntakeToFarScore;
    public PathChain blueFarWallToIntakeLoadingStation;
    public PathChain blueFarLoadingStationToFarScore;

    public PathChain blueFarScoreToPark;
    public PathChain blueLoadingStationToPark;

    public PathChain blueNearShootToIntakeFar;

    // Gate paths

    public PathChain blueOpenGate;
    public PathChain blueParkGate;



    public NovaPyraRobot robot;

    public BlueFarFromGoalConstants(Follower follower, NovaPyraRobot robot) {
        this.robot = robot;
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
                .setBrakingStrength(0.025)
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
                .3)
                .addParametricCallback(0.25, () -> {
                    robot.shotController.setState(ShotCycleState.INTAKE);
                })
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
                .setBrakingStrength(0.035)
                .addParametricCallback(0.50, () -> {
                    robot.shotController.setState(ShotCycleState.PREPARE_SHOT);
                })
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
                .addParametricCallback(0.25, () -> {
                    robot.shotController.setState(ShotCycleState.INTAKE);
                })
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
                .setBrakingStrength(0.035)
                .addParametricCallback(0.40, () -> {
                    robot.shotController.setState(ShotCycleState.PREPARE_SHOT);
                })
                .build();

        blueFarWallToIntakeLoadingStation = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                blueFarWallShoot,
                                blueFarWallIntakeLoadingStationCP,
                                blueFarWallIntakeLoadingStation
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueFarWallShootHeading),
                        Math.toRadians(blueLoadingZoneIntakeHeading),
                        0.5)
                .addParametricCallback(0.20, () -> {
                    robot.shotController.setState(ShotCycleState.INTAKE);
                })
                .build();

        blueFarLoadingStationToFarScore = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                blueFarWallIntakeLoadingStation,
                                blueFarWallIntakeLoadingStationCP,
                                blueFarWallShoot
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueLoadingZoneIntakeHeading),
                        Math.toRadians(blueFarWallShootHeading),
                        0.5)
                .setBrakingStrength(0.035)
                .addParametricCallback(0.30, () -> {
                    robot.shotController.setState(ShotCycleState.PREPARE_SHOT);
                })
                .build();

        blueLoadingStationToPark = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                blueFarWallIntakeLoadingStation,
                                blueFarWallParkPose
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueLoadingZoneIntakeHeading),
                        Math.toRadians(90.0),
                        0.5)
                .build();

        blueFarScoreToPark = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                blueFarWallShoot,
                                blueFarWallParkPose
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueFarWallShootHeading),
                        Math.toRadians(90.0),
                        0.5)
                .build();

        // Gate paths


        blueOpenGate = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                blueFarWallMidIntake,
                                blueGateOpenCP,
                                blueGateOpen
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueFarWallRowIntakeHeading),
                        Math.toRadians(blueGateOpenHeading),
                        0.5)
                .build();


        blueNearShootToIntakeFar = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(54,84),
                                blueFarWallFarIntake
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(132.0),
                        Math.toRadians(blueFarWallRowIntakeHeading),
                        0.5)
                .addParametricCallback(0.20, () -> {
                    robot.shotController.setState(ShotCycleState.INTAKE);
                })
                .build();


    }
}