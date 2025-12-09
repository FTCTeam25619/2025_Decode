package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.autos.RedFarFromGoalConstants;
import org.firstinspires.ftc.teamcode.constants.Alliance;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.lib.orchestration.PresetShot;
import org.firstinspires.ftc.teamcode.lib.orchestration.ShotCycleState;
import org.firstinspires.ftc.teamcode.lib.triggers.SupplierTrigger;
import org.firstinspires.ftc.teamcode.triggers.RobotTriggers;

// TODO: Convert sample auto code to real path. See AutoPaths for a path copied from Pedro Visualizer.

@Autonomous(name = "Red 6-Score Far Wall", group = "Autonomous")
@Configurable // Panels
public class RedAuto6ScoreFar extends CommandOpMode {
    private NovaPyraRobot robot;
    private RobotTriggers robotTriggers;
    private Follower follower;
    private RedFarFromGoalConstants paths;
    protected TelemetryData telemetryData;
    private Alliance alliance;

    // Mechanism commands - replace these with your actual subsystem commands
    private InstantCommand setupFarWallPresetShot() {
        return new InstantCommand(() -> {
            robot.shotController.setupPresetShot(PresetShot.FarWall);
        });
    }

    private InstantCommand intake() {
        return new InstantCommand(() -> {
            robot.shotController.setState(ShotCycleState.INTAKE);
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

    private InstantCommand spinDown() {
        return new InstantCommand(() -> {
            robot.shotController.setState(ShotCycleState.OFF);
        });
    }

    private InstantCommand alive() {
        return new InstantCommand(() -> {
            telemetry.addData("Path finished", true);
            telemetry.update();
        });
    }

    private Trigger readyToShootPreload() {
        return new SupplierTrigger(() -> (!follower.isBusy()) && robot.shotController.readyToShoot());
    }

    @Override
    public void initialize() {
        super.reset();

        alliance = Alliance.RED;

        telemetry.addData("Robot Status", "Initializing AutoScorePreloadFar " + alliance.name());
        telemetry.update();

        // Setup Dashboard if setup for shop debugging
        if (Constants.DebugModes.ENABLE_PANELS_DASHBOARD) {
            telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
            this.telemetryData = new TelemetryData(telemetry);
        }

        this.robot = new NovaPyraRobot(hardwareMap, telemetry, alliance);
        this.robotTriggers = new RobotTriggers(robot);
        this.robotTriggers.setupNonControllerTriggers();

        this.follower = robot.pedroFollower;
        this.paths = new RedFarFromGoalConstants(follower);

        // Initialize follower
        follower.setStartingPose(paths.redFarWallStart.withHeading(Math.toRadians(paths.redFarWallStartHeading)));

        // Schedule the autonomous sequence
        schedule(
                new ParallelCommandGroup(
                        new RunCommand(follower::update),
                        new SequentialCommandGroup(
                                // Score preload
                                setupFarWallPresetShot(),
                                prepareShot(),
                                new FollowPathCommand(follower, paths.redWallToFarZonePrescore),
                                alive(),
                                new WaitCommand(500),
                                shoot(),
                                new WaitCommand(3500),
                                intake(),
                                new FollowPathCommand(follower, paths.redFarZonePrescoreToFarIntake),
                                new WaitCommand(250),
                                setupFarWallPresetShot(),
                                prepareShot(),
                                new FollowPathCommand(follower, paths.redFarIntakeToFarZoneScore),
                                new WaitCommand(500),
                                shoot(),
                                new WaitCommand(3500),
                                // Park
                                new FollowPathCommand(follower, paths.redPark, false),
                                spinDown()
                        )
                )
        );

//        readyToShootPreload().whenActive(shoot());

        telemetry.addData("Robot Status", "Initialized AutoScorePreloadFar " + alliance.name());
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        this.robot.shotController.updateState();

        if (follower.getCurrentPath() != null) {
            telemetry.addData("Follower busy", follower.isBusy());
            telemetry.addData("Follower param end", follower.getCurrentPath().isAtParametricEnd());
            telemetry.addData("Follower heading error", follower.getHeadingError());
        }

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();
    }
}
