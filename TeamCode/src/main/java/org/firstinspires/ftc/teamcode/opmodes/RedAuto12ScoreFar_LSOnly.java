package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.autos.RedFarFromGoalConstants;
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

@Autonomous(name = "Red Far 12 Only Loading Station", group = "Autonomous")
@Configurable // Panels
public class RedAuto12ScoreFar_LSOnly extends CommandOpMode {
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

    private InstantCommand idleState() {
        return new InstantCommand(() -> {
            robot.shotController.setState(ShotCycleState.IDLE);
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

        telemetry.addData("Robot Status", "Initializing Auto Red 12 Far LS-LS-LS " + alliance.name());
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

        robot.vision.setTargetingTag(
                alliance == Alliance.BLUE ?
                        VisionConstants.AprilTags.BlueGoal :
                        VisionConstants.AprilTags.RedGoal
        );

        this.follower = robot.pedroFollower;
        this.paths = new RedFarFromGoalConstants(follower, robot);

        // Initialize follower
        follower.setStartingPose(paths.redFarWallStart.withHeading(Math.toRadians(paths.redFarWallStartHeading)));
        BlackboardHelpers.setAutoEndPose(blackboard, follower.getPose());

        // Schedule the autonomous sequence
        schedule(
                new ParallelCommandGroup(
                        new RunCommand(follower::update),
                        new SequentialCommandGroup(

                                /**
                                 FIRST CYCLE - Preload
                                 **/

                                // Score preload
                                // Prepare shot before starting
                                setupFarWallPresetShot(),
                                new FollowPathCommand(follower, paths.redWallToFarZonePrescore, 0.5).beforeStarting(prepareShot()),
                                new ParallelRaceGroup(
                                        new WaitCommand(TimingConstants.waitBeforeShootingMS), // 0.700 second timeout
                                        new WaitUntilCommand(() -> robot.shotController.readyToShoot()) // wait until shooter is ready
                                ),
                                shoot(),
                                new WaitCommand(TimingConstants.waitForShootingMS).andThen(idleState()),


                                /**
                                 SECOND CYCLE  - intake Loading Station
                                 **/

                                // Intake from loading station -- intake starts after 25% of path complete
                                new FollowPathCommand(follower, paths.redFarWallToIntakeLoadingStation, false),
                                new WaitCommand(TimingConstants.waitForIntakeLoadingStationMS),

                                // Score -- preparing shot after 30% of path complete. Begin shooting after 90% of path complete
                                new FollowPathCommand(follower, paths.redFarLoadingStationToFarScore).andThen(shoot()),
                                new WaitCommand(TimingConstants.waitForShootingMS),

                                /**
                                 THIRD CYCLE - intake Loading Station
                                 **/

                                // Intake from loading station -- intake starts after 25% of path complete
                                new FollowPathCommand(follower, paths.redFarWallToIntakeLoadingStation, false),
                                new WaitCommand(TimingConstants.waitForIntakeLoadingStationMS),

                                // Score -- preparing shot after 30% of path complete. Begin shooting after 90% of path complete
                                new FollowPathCommand(follower, paths.redFarLoadingStationToFarScore).andThen(shoot()),
                                new WaitCommand(TimingConstants.waitForShootingMS),

                                /**
                                 FOURTH CYCLE - intake Loading Station
                                 **/

                                // Intake from loading station -- intake starts after 25% of path complete
                                new FollowPathCommand(follower, paths.redFarWallToIntakeLoadingStation, false),
                                new WaitCommand(TimingConstants.waitForIntakeLoadingStationMS),

                                // Score -- preparing shot after 30% of path complete. Begin shooting after 90% of path complete
                                new FollowPathCommand(follower, paths.redFarLoadingStationToFarScore).andThen(shoot()),
                                new WaitCommand(TimingConstants.waitForShootingMS).andThen(spinDown()),
                                new FollowPathCommand(follower, paths.redFarScoreToPark, false)

                        )
                )
        );

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
        PanelsUtils.Drawing.drawPath(paths.redWallToFarZonePrescore);
        PanelsUtils.Drawing.drawPath(paths.redFarZonePrescoreToFarIntake);
        PanelsUtils.Drawing.drawPath(paths.redFarWallIntakeToFarScore);
        PanelsUtils.Drawing.drawPath(paths.redFarZoneScoreToMidIntake);
        PanelsUtils.Drawing.drawPath(paths.redMidWallIntakeToFarScore);
        PanelsUtils.Drawing.drawPath(paths.redFarWallToIntakeLoadingStation);
    }
}
