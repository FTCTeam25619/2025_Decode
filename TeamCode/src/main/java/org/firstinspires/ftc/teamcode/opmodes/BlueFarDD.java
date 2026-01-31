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
import org.firstinspires.ftc.teamcode.autos.BlueFarDDConstants;
import org.firstinspires.ftc.teamcode.autos.TimingConstants;
import org.firstinspires.ftc.teamcode.constants.Alliance;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.lib.orchestration.PresetShot;
import org.firstinspires.ftc.teamcode.lib.orchestration.ShotCycleState;
import org.firstinspires.ftc.teamcode.lib.panels.PanelsUtils;
import org.firstinspires.ftc.teamcode.lib.triggers.SupplierTrigger;
import org.firstinspires.ftc.teamcode.lib.util.BlackboardHelpers;
import org.firstinspires.ftc.teamcode.triggers.RobotTriggers;

//@Autonomous(name = "Blue Far 12 DD", group = "Autonomous")
@Configurable // Panels
public class BlueFarDD extends CommandOpMode {
    private NovaPyraRobot robot;
    private RobotTriggers robotTriggers;
    private Follower follower;
    private BlueFarDDConstants paths;
    protected TelemetryData telemetryData;
    private Alliance alliance;

    // Mechanism commands
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

    private Trigger readyToShootPreload() {
        return new SupplierTrigger(() -> (!follower.isBusy()) && robot.shotController.readyToShoot());
    }

    @Override
    public void initialize() {
        super.reset();

        alliance = Alliance.BLUE;

        telemetry.addData("Robot Status", "Initializing Auto 12 Ball Far " + alliance.name());
        telemetry.update();

        // Setup Dashboard if setup for shop debugging
        if (Constants.DebugModes.ENABLE_PANELS_DASHBOARD) {
            telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
            this.telemetryData = new TelemetryData(telemetry);
            PanelsUtils.Drawing.init();
            PanelsUtils.Drawing.sendPacket();
        }

        this.robot = new NovaPyraRobot(hardwareMap, telemetry, alliance);
        this.robotTriggers = new RobotTriggers(robot);
        this.robotTriggers.setupNonControllerTriggers();

        this.follower = robot.pedroFollower;
        this.paths = new BlueFarDDConstants(follower);

        robot.vision.setTargetingTag(
                alliance == Alliance.BLUE ?
                        VisionConstants.AprilTags.BlueGoal :
                        VisionConstants.AprilTags.RedGoal
        );

        // Initialize follower
        follower.setStartingPose(paths.startPose.withHeading(paths.startPose.getHeading()));
        BlackboardHelpers.setAutoEndPose(blackboard, follower.getPose());

        // Schedule the autonomous sequence
        schedule(
                new ParallelCommandGroup(
                        new RunCommand(follower::update),
                        new SequentialCommandGroup(
                                // Score preload
                                setupFarWallPresetShot(),
                                prepareShot(),
                                new FollowPathCommand(follower, paths.preloadScore),
                                new WaitCommand(TimingConstants.waitBeforeShootingMS),
                                shoot(),
                                new WaitCommand(TimingConstants.waitForShootingMS),
                                // Intake 3 from mid row
                                intake(),
                                new FollowPathCommand(follower, paths.intakeMid),
                                new WaitCommand(TimingConstants.waitForIntakeMS),
                                // Score 3
                                setupFarWallPresetShot(),
                                prepareShot(),
                                new FollowPathCommand(follower, paths.scoreMid),
                                new WaitCommand(TimingConstants.waitBeforeShootingMS),
                                shoot(),
                                new WaitCommand(TimingConstants.waitForShootingMS),
                                // Intake 3 from close (far) row
                                intake(),
                                new FollowPathCommand(follower, paths.intakeNear),
                                new WaitCommand(TimingConstants.waitForIntakeMS),
                                // Score 3
                                setupFarWallPresetShot(),
                                prepareShot(),
                                new FollowPathCommand(follower, paths.scoreNear),
                                new WaitCommand(TimingConstants.waitBeforeShootingMS),
                                shoot(),
                                new WaitCommand(TimingConstants.waitForShootingMS),
                                // Intake 3 from corner
                                intake(),
                                new FollowPathCommand(follower, paths.intakeCorner),
                                new WaitCommand(TimingConstants.waitForIntakeMS),
                                // Score 3
                                setupFarWallPresetShot(),
                                prepareShot(),
                                new FollowPathCommand(follower, paths.scoreCorner),
                                new WaitCommand(TimingConstants.waitBeforeShootingMS),
                                shoot(),
                                new WaitCommand(TimingConstants.waitForShootingMS),
                                // Intake 3 from corner
                                intake(),
                                new FollowPathCommand(follower, paths.intakeCorner),
                                new WaitCommand(TimingConstants.waitForIntakeMS),
                                // Score 3
                                setupFarWallPresetShot(),
                                prepareShot(),
                                new FollowPathCommand(follower, paths.scoreCorner),
                                new WaitCommand(TimingConstants.waitBeforeShootingMS),
                                shoot(),
                                new WaitCommand(TimingConstants.waitForShootingMS),
                                // Park
                                new FollowPathCommand(follower, paths.park, false),
                                spinDown()
                        )
                )
        );

//        readyToShootPreload().whenActive(shoot());

        telemetry.addData("Robot Status", "Initialized Auto 12 Ball Far " + alliance.name());
        telemetry.update();
    }

    public void initialize_loop() {
        super.initialize_loop();
        // Things that need to run while disabled every cycle
        if (Constants.DebugModes.ENABLE_PANELS_DASHBOARD) {
            PanelsUtils.Drawing.drawVisionAndCurrent(robot.vision, robot.pedroFollower);
            drawPaths();
            PanelsUtils.Drawing.sendPacket();
        }
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

        if (Constants.DebugModes.ENABLE_PANELS_DASHBOARD) {
            PanelsUtils.Drawing.drawVisionAndCurrentAndHistory(robot.vision, robot.pedroFollower);
            drawPaths();
            PanelsUtils.Drawing.sendPacket();
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

    private void drawPaths() {
        PanelsUtils.Drawing.drawPath(paths.preloadScore);
        PanelsUtils.Drawing.drawPath(paths.intakeMid);
        PanelsUtils.Drawing.drawPath(paths.scoreMid);
        PanelsUtils.Drawing.drawPath(paths.intakeNear);
        PanelsUtils.Drawing.drawPath(paths.scoreNear);
        PanelsUtils.Drawing.drawPath(paths.intakeCorner);
        PanelsUtils.Drawing.drawPath(paths.scoreCorner);
        PanelsUtils.Drawing.drawPath(paths.park);
    }
}
