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
public class RedFarFromGoalConstants {
    // Shared poses (used by both regular and gate paths)
    public Pose redFarWallStart = new Pose(88.000, 8.000);
    public Pose redFarWallToShootCP = new Pose(87.735, 12.397);
    public Pose redFarWallShoot = new Pose(87.258, 14.543);
    public Pose redFarWallShootToFarIntakeCP = new Pose(78.914, 35.0);
    public Pose redFarWallFarIntake = new Pose(130.25, 35.0);
    public Pose redFarWallShootToMidIntakeCP = new Pose(78.676, 72.238);
    public Pose redFarWallMidIntake = new Pose(130.25, 55.192);
    public Pose redFarWallIntakeLoadingStationCP = new Pose(93, 25);
    public Pose redFarWallIntakeLoadingStation = new Pose(132.5, 7.75);
    public Pose redFarWallParkPose = new Pose(114.00, 46.00);

    // Gate-specific poses

    public Pose redGateOpenCP = new Pose(120.679, 62.764);
    public Pose redGateOpen = new Pose(127.5, 65.000);
    public Pose redWallIntakeGate = new Pose(129.0, 58.0);
    // Shared headings
    public double redFarWallStartHeading = 90.0;
    public double redFarWallFirstShotHeading = 69.0;
    public double redFarWallShootHeading = 69.0;
    public double redFarWallRowIntakeHeading = 180.0;
    public double redFarWallMidIntakeHeading = 180.0;
    public double redLoadingZoneIntakeHeading = 180.0;

    // Gate-specific headings
    public double redGateOpenHeading = 90.0;
    public double redGateParkHeading = 90.0;

    public PathChain redWallToFarZonePrescore;
    public PathChain redFarZonePrescoreToFarIntake;
    public PathChain redFarWallIntakeToFarScore;
    public PathChain redFarZoneScoreToMidIntake;
    public PathChain redMidWallIntakeToFarScore;
    public PathChain redFarWallToIntakeLoadingStation;
    public PathChain redFarLoadingStationToFarScore;

    public PathChain redFarScoreToPark;
    public PathChain redLoadingStationToPark;

    public PathChain redNearShootToIntakeFar;

    // Gate paths

    public PathChain redOpenGate;
    public PathChain redParkGate;



    public NovaPyraRobot robot;

    public RedFarFromGoalConstants(Follower follower, NovaPyraRobot robot) {
        this.robot = robot;
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
                .setBrakingStrength(0.025)
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
                        .3)
                .addParametricCallback(0.25, () -> {
                    robot.shotController.setState(ShotCycleState.INTAKE);
                })
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
                        .5)
                .setBrakingStrength(0.035)
                .addParametricCallback(0.50, () -> {
                    robot.shotController.setState(ShotCycleState.PREPARE_SHOT);
                })
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
                        .5)
                .addParametricCallback(0.25, () -> {
                    robot.shotController.setState(ShotCycleState.INTAKE);
                })
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
                .setBrakingStrength(0.035)
                .addParametricCallback(0.40, () -> {
                    robot.shotController.setState(ShotCycleState.PREPARE_SHOT);
                })
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
                .setLinearHeadingInterpolation(
                        Math.toRadians(redFarWallShootHeading),
                        Math.toRadians(redLoadingZoneIntakeHeading),
                        0.5)
                .addParametricCallback(0.20, () -> {
                    robot.shotController.setState(ShotCycleState.INTAKE);
                })
                .build();

        redFarLoadingStationToFarScore = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                redFarWallIntakeLoadingStation,
                                redFarWallIntakeLoadingStationCP,
                                redFarWallShoot
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(redLoadingZoneIntakeHeading),
                        Math.toRadians(redFarWallShootHeading),
                        0.5)
                .setBrakingStrength(0.035)
                .addParametricCallback(0.30, () -> {
                    robot.shotController.setState(ShotCycleState.PREPARE_SHOT);
                })
                .build();

        redLoadingStationToPark = follower
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

        redFarScoreToPark = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                redFarWallShoot,
                                redFarWallParkPose
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(redFarWallShootHeading),
                        Math.toRadians(90.0),
                        0.5)
                .build();

        // Gate paths


        redOpenGate = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                redFarWallMidIntake,
                                redGateOpenCP,
                                redGateOpen
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(redFarWallRowIntakeHeading),
                        Math.toRadians(redGateOpenHeading),
                        0.5)
                .build();


        redNearShootToIntakeFar = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(90, 84),
                                redFarWallFarIntake
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(48.0),
                        Math.toRadians(redFarWallRowIntakeHeading),
                        0.5)
                .addParametricCallback(0.20, () -> {
                    robot.shotController.setState(ShotCycleState.INTAKE);
                })
                .build();


    }
}