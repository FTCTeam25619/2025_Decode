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
public class BlueNearGoalConstants {

    public Pose blueNearWallStart = new Pose(15.75, 112.3);
    public Pose blueNearWallToShootCP = new Pose(41.5, 112.0);
    public Pose blueNearWallShoot = new Pose(54.0, 84.0);
    public Pose blueNearWallShootToNearIntakeCP = new Pose(63.193, 63.615);
    public Pose blueNearWallNearIntake = new Pose(18.5, 86.532);
    public Pose blueNearWallShootToMidIntakeCP = new Pose(54.0, 53.0);
    public Pose blueNearWallMidIntake = new Pose(12.0, 58.0);
    public Pose blueNearWallPark = new Pose(54.606, 114.275);

    public Pose blueGateOpenPose = new Pose(14.5, 61);
    public Pose blueGateOpenPoseCP = new Pose (41,64.25);
    public Pose blueGateIntakePose = new Pose(12.5,50.0);
    public Pose blueGateIntakePoseCP = new Pose(16.6,55.0);

    public double blueGateOpenHeading = 330;
    public double blueGateIntakeHeading = 320;
    public double blueNearWallStartHeading = 90.0;
    public double blueNearWallShootHeading = 133.0;
    public double blueNearWallRowIntakeHeading = 0.0;
    public double blueNearWallMidRowIntakeHeading = 0.0;


    public PathChain blueWallToNearZonePrescore;
    public PathChain blueNearZoneToNearIntakeblueBlue;
    public PathChain blueNearWallIntakeToNearScore;
    public PathChain blueNearZoneScoreToMidIntake;
    public PathChain blueMidWallIntakeToNearScore;

    public PathChain blueNearShootToIntakeFromGate;

    public PathChain blueScoreToOpenGate;
    public PathChain blueOpenGateToIntakeGate;

    public PathChain blueNearWallToPark;

    public NovaPyraRobot robot;
    public BlueNearGoalConstants(Follower follower, NovaPyraRobot robot) {
        this.robot = robot;
        buildPaths(follower);
    }

    private void buildPaths(Follower follower) {
        blueWallToNearZonePrescore = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                blueNearWallStart,
                                blueNearWallToShootCP,
                                blueNearWallShoot
                        )
                )
                .addParametricCallback(0.90, () -> {
                    robot.shotController.setState(ShotCycleState.SHOOTING);
                })
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueNearWallStartHeading),
                        Math.toRadians(blueNearWallShootHeading),
                        0.7)
                .build();

        blueNearZoneToNearIntakeblueBlue = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                blueNearWallShoot,
                                //blueNearWallShootToNearIntakeCP,
                                blueNearWallNearIntake
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueNearWallShootHeading),
                        Math.toRadians(blueNearWallRowIntakeHeading),
                0.1)
                .addParametricCallback(.1, () -> {
                    robot.shotController.setState(ShotCycleState.INTAKE);
                })
                .build();

        blueNearWallIntakeToNearScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                blueNearWallNearIntake,
                                blueNearWallShoot
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueNearWallRowIntakeHeading),
                        Math.toRadians(blueNearWallShootHeading),
                0.7)
                .addParametricCallback(.30, () -> {
                    robot.shotController.autoTargetShot();
                    robot.shotController.setState(ShotCycleState.PREPARE_SHOT);
                })
                .addParametricCallback(0.90, () -> {
                    robot.shotController.setState(ShotCycleState.SHOOTING);
                })
                .build();

        blueNearZoneScoreToMidIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                blueNearWallShoot,
                                blueNearWallShootToMidIntakeCP,
                                blueNearWallMidIntake
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueNearWallShootHeading),
                        Math.toRadians(blueNearWallMidRowIntakeHeading),
                        0.4)
                .addParametricCallback(0.25, () -> {
                    robot.shotController.setState(ShotCycleState.INTAKE);
                })
                .build();

        blueMidWallIntakeToNearScore = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                blueNearWallMidIntake,
                                blueNearWallShootToMidIntakeCP,
                                blueNearWallShoot
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueNearWallRowIntakeHeading),
                        Math.toRadians(blueNearWallShootHeading),
                        0.7)
                .addParametricCallback(0.30, () -> {
                    robot.shotController.autoTargetShot();
                    robot.shotController.setState(ShotCycleState.PREPARE_SHOT);
                })
                .addParametricCallback(0.90, () -> {
                    robot.shotController.setState(ShotCycleState.SHOOTING);
                })
                .build();

        blueNearWallToPark = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                blueNearWallShoot,
                                blueNearWallPark
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueNearWallShootHeading),
                        Math.toRadians(blueNearWallShootHeading),
                        0.7)
                .build();


        blueScoreToOpenGate = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                blueNearWallShoot,
                                blueGateOpenPoseCP,
                                blueGateOpenPose
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueNearWallShootHeading),
                        Math.toRadians(blueGateOpenHeading),
                        0.5)
                .build();

        blueOpenGateToIntakeGate = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                blueGateOpenPose,
                                blueGateIntakePoseCP,
                                blueGateIntakePose
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueGateOpenHeading),
                        Math.toRadians(blueGateIntakeHeading),
                        0.5)
                .build();

        blueNearShootToIntakeFromGate = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                blueNearWallShoot,
                                blueNearWallShootToNearIntakeCP,
                                blueGateIntakePose
                        )
                )
                .addParametricCallback(0.25, () -> {
                    robot.shotController.setState(ShotCycleState.INTAKE);
                })
                .setLinearHeadingInterpolation(
                        Math.toRadians(blueNearWallShootHeading),
                        Math.toRadians(0.0),
                        0.5)
                .build();
    }
}
