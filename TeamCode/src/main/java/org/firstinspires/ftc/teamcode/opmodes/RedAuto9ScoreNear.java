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
import org.firstinspires.ftc.teamcode.autos.RedNearGoalConstants;
import org.firstinspires.ftc.teamcode.autos.TimingConstants;
import org.firstinspires.ftc.teamcode.constants.Alliance;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.lib.orchestration.PresetShot;
import org.firstinspires.ftc.teamcode.lib.orchestration.ShotCycleState;
import org.firstinspires.ftc.teamcode.lib.triggers.SupplierTrigger;
import org.firstinspires.ftc.teamcode.lib.util.BlackboardHelpers;
import org.firstinspires.ftc.teamcode.triggers.RobotTriggers;

@Autonomous(name = "Red 9-Score Near Goal", group = "Autonomous")
@Configurable // Panels
public class RedAuto9ScoreNear extends CommandOpMode {
    private NovaPyraRobot robot;
    private RobotTriggers robotTriggers;
    private Follower follower;
    private RedNearGoalConstants paths;
    protected TelemetryData telemetryData;
    private Alliance alliance;

    // Mechanism commands - replace these with your actual subsystem commands
    private InstantCommand setupNearWallPresetShot() {
        return new InstantCommand(() -> {
            robot.shotController.setupPresetShot(PresetShot.NearZoneMid);
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

    private Trigger readyToShootPreload() {
        return new SupplierTrigger(() -> (!follower.isBusy()) && robot.shotController.readyToShoot());
    }

    @Override
    public void initialize() {
        super.reset();

        alliance = Alliance.RED;

        telemetry.addData("Robot Status", "Initializing Auto 9 Ball Near " + alliance.name());
        telemetry.update();

        // Setup Dashboard if setup for shop debugging
        if (Constants.DebugModes.ENABLE_PANELS_DASHBOARD) {
            telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
            this.telemetryData = new TelemetryData(telemetry);
        }

        this.robot = new NovaPyraRobot(hardwareMap, telemetry, alliance);
        this.robotTriggers = new RobotTriggers(robot);
        this.robotTriggers.setupNonControllerTriggers();

        robot.vision.setTargetingTag(
                alliance == Alliance.BLUE ?
                        VisionConstants.AprilTags.BlueGoal :
                        VisionConstants.AprilTags.RedGoal
        );

        this.follower = robot.pedroFollower;
        this.paths = new RedNearGoalConstants(follower);

        // Initialize follower
        follower.setStartingPose(paths.redNearWallStart.withHeading(Math.toRadians(paths.redNearWallStartHeading)));
        BlackboardHelpers.setAutoEndPose(blackboard, follower.getPose());

        // Schedule the autonomous sequence
        schedule(
                new ParallelCommandGroup(
                        new RunCommand(follower::update),
                        new SequentialCommandGroup(

                                // Score preload
                                prepareShot(),
                                new FollowPathCommand(follower, paths.redWallToNearZonePrescore),
                                new WaitCommand(TimingConstants.waitBeforeShootingMS),
                                shoot(),
                                new WaitCommand(TimingConstants.waitForShootingMS),

                                // Intake 3 from row closest to far wall from goal
                                intake(),
                                new FollowPathCommand(follower, paths.redNearZonePrescoreToNearIntake),

                                // Score 3
                                prepareShot(),
                                new FollowPathCommand(follower, paths.redNearWallIntakeToNearScore),
                                new WaitCommand(TimingConstants.waitBeforeShootingMS),
                                shoot(),
                                new WaitCommand(TimingConstants.waitForShootingMS),

                                // Intake 3 from from mid row
                                intake(),
                                new FollowPathCommand(follower, paths.redNearZoneScoreToMidIntake),

                                // Score 3
                                prepareShot(),
                                new FollowPathCommand(follower, paths.redMidWallIntakeToNearScore),
                                new WaitCommand(TimingConstants.waitBeforeShootingMS),
                                shoot(),
                                new WaitCommand(TimingConstants.waitForShootingMS),

                                // Park
                                new FollowPathCommand(follower, paths.redNearWallToPark, false),
                                spinDown()
                        )
                )
        );

//        readyToShootPreload().whenActive(shoot());

        telemetry.addData("Robot Status", "Initialized Auto 9 Ball Near " + alliance.name());
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        this.robot.shotController.updateState();

        if (Constants.DebugModes.Telemetry.DEBUG_PATH_FOLLOWING && follower.getCurrentPath() != null) {
            telemetry.addData("Follower busy", follower.isBusy());
            telemetry.addData("Follower param end", follower.getCurrentPath().isAtParametricEnd());
            telemetry.addData("Follower heading error", follower.getHeadingError());
        }

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();
    }

    @Override
    public void end() {
        super.end();
        BlackboardHelpers.setAutoEndPose(blackboard, follower.getPose());
    }
}
