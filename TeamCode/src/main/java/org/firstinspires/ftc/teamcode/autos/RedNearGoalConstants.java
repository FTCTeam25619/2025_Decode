package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.lib.orchestration.ShotCycleState;

@Configurable
public class RedNearGoalConstants {

    // All x values mirrored: new_x = 144 - old_x, y unchanged
    public Pose redNearWallStart = new Pose(126.856, 123.233);
    public Pose redNearWallToShootCP = new Pose(102.5, 112.0);
    public Pose redNearFirstShoots = new Pose(97.0, 99.5);
    public Pose redNearWallShoot = new Pose(90.0, 84.0);
    public Pose redNearWallShootPark = new Pose(90.0, 114.0);
    public Pose redNearWallShootToNearIntakeCP = new Pose(80.807, 63.615);
    public Pose redNearWallNearIntake = new Pose(126.0, 86.532);
    public Pose redNearWallShootToMidIntakeCP = new Pose(90.0, 53.0);
    public Pose redNearWallMidIntake = new Pose(131.5, 58.0);
    public Pose redNearWallPark = new Pose(89.394, 114.275);

    public Pose redParkByGate = new Pose(120.0, 73.0);

    public Pose redGateOpenPose = new Pose(129.5, 61.0);
    public Pose redGateOpenPoseCP = new Pose(103.0, 64.25);
    public Pose redGateIntakePose = new Pose(131.5, 50.0);
    public Pose redGateIntakePoseCP = new Pose(127.4, 55.0);
    public Pose redNearWallShootClose = new Pose(94.899, 117.769);

    public Pose redGateOpenIntakePose = new Pose(130.0, 59.7);

    public Pose redSweepIntakePoseCP1 = new Pose(118.78, 34.22);
    public Pose redSweepIntakePoseCP2 = new Pose(138.1, 98.8);

    public Pose redSweepEndPose = new Pose(131.0, 9.0);

    // Headings mirrored: new_heading = (180 - old_heading + 360) % 360
    public double redNearWallShootCloseHeading = 35.0;       // 180 - 145

    public double redGateOpenIntakeHeading = 208.6;          // (180 - 331.4 + 360) % 360

    public double redGateOpenHeading = 210.0;                // (180 - 330 + 360) % 360
    public double redGateIntakeHeading = 220.0;              // (180 - 320 + 360) % 360
    public double redNearWallStartHeading = 36.52;           // 180 - 143.48
    public double redNearWallShootHeading = 45.0;            // 180 - 135
    public double redNearWallShootParkHeading = 23.0;        // 180 - 157
    public double redNearWallLastShootHeading = 48.0;        // 180 - 132
    public double redNearWallRowIntakeHeading = 180.0;       // (180 - 0 + 360) % 360
    public double redSweepIntakeHeading = 140.0;             // (180 - 40 + 360) % 360
    public double redNearWallMidRowIntakeHeading = 180.0;    // (180 - 0 + 360) % 360

    // Target-Poses (red goal, mirrored from blue goal at (3, 142))
    public double goalTargetX = 141.0;
    public double goalTargetY = 142.0;

    public PathChain redWallToNearZonePrescore;
    public PathChain redNearZoneToNearIntake;
    public PathChain redNearWallIntakeToNearScore;
    public PathChain redNearWallIntakeToNearScoreAndPark;
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

    public PathChain redOpenAndIntakeGate;

    public NovaPyraRobot robot;

    public RedNearGoalConstants(Follower follower, NovaPyraRobot robot) {
        this.robot = robot;
        buildPaths(follower);
    }

    private void buildPaths(Follower follower) {
        redWallToNearZonePrescore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                redNearWallStart,
                                redNearFirstShoots
                        )
                )
                .addParametricCallback(.1, () -> {
                    robot.shotController.setState(ShotCycleState.SHOOTING);
                })
                .setConstantHeadingInterpolation(
                        Math.toRadians(redNearWallStartHeading)
                )
                .build();

        redNearZoneToNearIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                redNearWallShoot,
                                redNearWallNearIntake
                        )
                )
                .setConstantHeadingInterpolation(
                        Math.toRadians(180.0))
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
                        Math.toRadians(redNearWallShootHeading),
                        0.5)
                .addParametricCallback(.30, () -> {
                    robot.shotController.setState(ShotCycleState.PREPARE_SHOT);
                })
                .addParametricCallback(0.95, () -> {
                    robot.shotController.setState(ShotCycleState.SHOOTING);
                })
                .build();

        redNearWallIntakeToNearScoreAndPark = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                redNearWallNearIntake,
                                redNearWallShootPark
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(redNearWallRowIntakeHeading),
                        Math.toRadians(redNearWallShootParkHeading),
                        0.5)
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
                                redNearFirstShoots,
                                redNearWallShootToMidIntakeCP,
                                redNearWallMidIntake
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(redNearWallStartHeading),
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
                ).setLinearHeadingInterpolation(
                        Math.toRadians(redNearWallRowIntakeHeading),
                        Math.toRadians(redNearWallLastShootHeading),
                        0.4)
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
                ).setLinearHeadingInterpolation(
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
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0.0,
                                        0.2,
                                        HeadingInterpolator.linear(
                                                Math.toRadians(redNearWallRowIntakeHeading),
                                                Math.toRadians(redNearWallShootCloseHeading)
                                        )
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.2,
                                        1.0,
                                        HeadingInterpolator.facingPoint(goalTargetX, goalTargetY)
                                )
                        )
                )
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

        redOpenAndIntakeGate = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                redNearWallShoot,
                                redGateOpenPoseCP,
                                redGateOpenIntakePose
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0.0),           // mirrored from 180°
                        Math.toRadians(redGateOpenIntakeHeading),
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
                        Math.toRadians(180.0),         // mirrored from 0°
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