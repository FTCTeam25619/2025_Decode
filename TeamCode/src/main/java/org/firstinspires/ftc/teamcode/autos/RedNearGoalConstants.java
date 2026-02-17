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
public class RedNearGoalConstants {

    public Pose redNearWallStart = new Pose(128.25, 114.8);
    public Pose redNearWallToShootCP = new Pose(102.5, 112.0);
    public Pose redNearWallShoot = new Pose(90.0, 84.0);
    public Pose redNearWallShootToNearIntakeCP = new Pose(80.807, 63.615);
    public Pose redNearWallNearIntake = new Pose(126.0, 86.532);
    public Pose redNearWallShootToMidIntakeCP = new Pose(90.0, 53.0);
    public Pose redNearWallMidIntake = new Pose(129.5, 58.0);
    public Pose redNearWallPark = new Pose(89.394, 114.275);

    public Pose redParkByGate = new Pose(120.0, 73);

    public Pose redGateOpenPose = new Pose(129.5, 61);
    public Pose redGateOpenPoseCP = new Pose(103, 64.25);
    public Pose redGateIntakePose = new Pose(131.5, 50.0);
    public Pose redGateIntakePoseCP = new Pose(127.4, 55.0);
    public Pose redNearWallShootClose = new Pose(94.899, 117.769);


    public Pose redSweepIntakePoseCP1 = new Pose(118.78, 34.22);
    public Pose redSweepIntakePoseCP2 = new Pose(138.1, 98.8);

    public Pose redSweepEndPose = new Pose(131.0, 9.0);

    public double redNearWallShootCloseHeading = 215.0;

    public double redGateOpenHeading = 210;
    public double redGateIntakeHeading = 220;
    public double redNearWallStartHeading = 90.0;
    public double redNearWallShootHeading = 48.0;
    public double redNearWallLastShootHeading = 46.0;
    public double redNearWallRowIntakeHeading = 180.0;
    public double redSweepIntakeHeading = 140.0;
    public double redNearWallMidRowIntakeHeading = 180.0;


    public PathChain redWallToNearZonePrescore;
    public PathChain redNearZoneToNearIntake;
    public PathChain redNearWallIntakeToNearScore;
    public PathChain redNearZoneScoreToMidIntake;
    public PathChain redMidWallIntakeToNearScore;
    public PathChain redNearShootToIntakeFromGate;
    public PathChain redNearIntakeToCloseShoot;
    public PathChain redScoreToOpenGate;
    public PathChain redOpenGateToIntakeGate;
    public PathChain redGateIntakeToNearShoot;
    public PathChain redNearWallToPark;
    public PathChain redNearShootToSweep;
    public PathChain redSweepToPark;

    public NovaPyraRobot robot;
    public RedNearGoalConstants(Follower follower, NovaPyraRobot robot) {
        this.robot = robot;
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
                .addParametricCallback(.95, () -> {
                    robot.shotController.setState(ShotCycleState.SHOOTING);
                })
                .setLinearHeadingInterpolation(
                        Math.toRadians(redNearWallStartHeading),
                        Math.toRadians(redNearWallShootHeading),
                        0.7)
                .build();

        redNearZoneToNearIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                redNearWallShoot,
                                //redNearWallShootToNearIntakeCP,
                                redNearWallNearIntake
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(redNearWallShootHeading),
                        Math.toRadians(redNearWallRowIntakeHeading),
                        0.1)
                .addParametricCallback(.1, () -> {
                    robot.shotController.setState(ShotCycleState.INTAKE);
                })
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
                        Math.toRadians(redNearWallLastShootHeading),
                        0.7)
                .addParametricCallback(.30, () -> {
                    robot.shotController.setState(ShotCycleState.PREPARE_SHOT);
                })
                .addParametricCallback(0.95, () -> {
                    robot.shotController.setState(ShotCycleState.SHOOTING);
                })
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
                .addParametricCallback(0.25, () -> {
                    robot.shotController.setState(ShotCycleState.INTAKE);
                })
                .build();

        redMidWallIntakeToNearScore = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                redNearWallMidIntake,
                                redNearWallShootToMidIntakeCP,
                                redNearWallShoot
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(redNearWallRowIntakeHeading),
                        Math.toRadians(redNearWallShootHeading),
                        0.5)
                .addParametricCallback(0.30, () -> {
                    robot.shotController.setState(ShotCycleState.PREPARE_SHOT);
                })
                .addParametricCallback(0.90, () -> {
                    robot.shotController.setState(ShotCycleState.SHOOTING);
                })
                .build();

        redGateIntakeToNearShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                redGateIntakePose,
                                redNearWallShootToMidIntakeCP,
                                redNearWallShoot
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(redGateIntakeHeading),
                        Math.toRadians(redNearWallShootHeading),
                        0.5)
                .addParametricCallback(0.30, () -> {
                    robot.shotController.setState(ShotCycleState.PREPARE_SHOT);
                })
                .addParametricCallback(0.90, () -> {
                    robot.shotController.setState(ShotCycleState.SHOOTING);
                })
                .build();


        redNearIntakeToCloseShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                redNearWallNearIntake,
                                redNearWallShootClose
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(redNearWallRowIntakeHeading),
                        Math.toRadians(redNearWallShootCloseHeading),
                        0.2)
                .addParametricCallback(0.20, () -> {
                    robot.shotController.setState(ShotCycleState.PREPARE_SHOT);
                })
                .addParametricCallback(0.80, () -> {
                    robot.shotController.setState(ShotCycleState.SHOOTING);
                })
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
                .build();


        redScoreToOpenGate = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                redNearWallShoot,
                                redGateOpenPoseCP,
                                redGateOpenPose
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(redNearWallShootHeading),
                        Math.toRadians(redGateOpenHeading),
                        0.5)
                .build();

        redOpenGateToIntakeGate = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                redGateOpenPose,
                                redGateIntakePoseCP,
                                redGateIntakePose
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(redGateOpenHeading),
                        Math.toRadians(redGateIntakeHeading),
                        0.5)
                .build();

        redNearShootToIntakeFromGate = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                redNearWallShoot,
                                redNearWallShootToNearIntakeCP,
                                redGateIntakePose
                        )
                )
                .addParametricCallback(0.25, () -> {
                    robot.shotController.setState(ShotCycleState.INTAKE);
                })
                .setLinearHeadingInterpolation(
                        Math.toRadians(redNearWallShootHeading),
                        Math.toRadians(180.0),
                        0.5)
                .build();

        redNearShootToSweep = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                redNearWallShoot,
                                redSweepIntakePoseCP1,
                                redSweepIntakePoseCP2,
                                redSweepEndPose
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(redNearWallShootHeading),
                        Math.toRadians(redSweepIntakeHeading),
                        0.2)
                .build();

        redSweepToPark = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                redSweepEndPose,
                                redParkByGate
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(redSweepIntakeHeading),
                        Math.toRadians(90.0),
                        0.2)
                .build();


    }
}