package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.autos.AutoPaths;
import org.firstinspires.ftc.teamcode.lib.orchestration.PresetShot;
import org.firstinspires.ftc.teamcode.lib.orchestration.ShotCycleState;
import org.firstinspires.ftc.teamcode.lib.triggers.SupplierTrigger;

// TODO: Convert sample auto code to real path. See AutoPaths for a path copied from Pedro Visualizer.

@Autonomous(name = "Score Preload Far Wall", group = "Autonomous")
@Configurable // Panels
public class AutoScorePreloadFar extends CommandOpMode {
    private final NovaPyraRobot robot;
    private final Follower follower;
    private final AutoPaths paths;
    protected final TelemetryData telemetryData;

    // Poses
    private final Pose startPose = new Pose(9, 111, Math.toRadians(-90));
    private final Pose scorePose = new Pose(16, 128, Math.toRadians(-45));
    private final Pose pickup1Pose = new Pose(30, 121, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(30, 131, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(45, 128, Math.toRadians(90));
    private final Pose parkPose = new Pose(68, 96, Math.toRadians(-90));

    // Path chains
    private PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3;
    private PathChain scorePickup1, scorePickup2, scorePickup3, park;

    private boolean pathCompleted = false;

    public AutoScorePreloadFar(NovaPyraRobot robot, Telemetry telemetry) {
        this.robot = robot;
        this.follower = robot.pedroFollower;
        this.paths = new AutoPaths(follower);
        this.telemetryData = new TelemetryData(telemetry);
    }

    public void buildPaths() {
        // FIXME: Do we need these?
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePose,
                        new Pose(68, 110), // Control point
                        parkPose)
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    // Mechanism commands - replace these with your actual subsystem commands
    private InstantCommand setupFarWallPresetShot() {
        return new InstantCommand(() -> {
            robot.shotController.setupPresetShot(PresetShot.FarWall);
        });
    }

    private InstantCommand prepareShot() {
        return new InstantCommand(() -> {
            robot.shotController.setState(ShotCycleState.PREPARE_SHOT);
        });
    }

    private InstantCommand shoot() {
        return new InstantCommand(() -> {
            robot.shotController.setState(ShotCycleState.SHOOTING);
        });
    }

    private InstantCommand setPathCompleteFlag(boolean flagValue) {
        return new InstantCommand(() -> { pathCompleted = flagValue; });
    }

    private Trigger readyToShootPreload() {
        return new SupplierTrigger(() -> pathCompleted && robot.shotController.readyToShoot());
    }

    @Override
    public void initialize() {
        super.reset();

        // Initialize follower
        follower.setStartingPose(startPose);
        buildPaths();

        // Schedule the autonomous sequence
        schedule(
                new RunCommand(follower::update),
                // Score preload
                setupFarWallPresetShot(),
                prepareShot(),
                setPathCompleteFlag(false),
                new FollowPathCommand(follower, paths.wallToFarZonePrescore),
                setPathCompleteFlag(true),
                new WaitCommand(1000),
                // Shoot will happen during the WaitCommand when triggered
                setPathCompleteFlag(false),
                // Park
                new FollowPathCommand(follower, park, false) // park with holdEnd false
        );

        readyToShootPreload().whenActive(shoot());
    }

    @Override
    public void run() {
        super.run();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();
    }
}
