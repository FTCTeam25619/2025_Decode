package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
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
import org.firstinspires.ftc.teamcode.autos.BlueFarFromGoalConstants;
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

@Autonomous(name = "Blue Far 12", group = "Autonomous")
@Configurable // Panels
public class BlueAuto12ScoreFar extends CommandOpMode {
    private NovaPyraRobot robot;
    private RobotTriggers robotTriggers;
    private Follower follower;
    private BlueFarFromGoalConstants paths;
    protected TelemetryData telemetryData;
    private Alliance alliance;


    private static boolean firstRun = true;

    // Mechanism commands - replace these with your actual subsystem commands
    private InstantCommand setupFarWallPresetShot() {
        return new InstantCommand(() -> {
            robot.shotController.setupPresetShot(PresetShot.FarWall);
        });
    }

    private InstantCommand setAutoTargetShot() {
        return new InstantCommand(() -> {
            robot.shotController.autoTargetShot();
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

    private Command turnToGoal() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    double dx = paths.goalTargetX - follower.getPose().getX();
                    double dy = paths.goalTargetY - follower.getPose().getY();
                    follower.constants.holdPointHeadingScaling = 0.85; // full power for turnTo
                    follower.turnTo(Math.atan2(dy, dx));
                }),
                new WaitUntilCommand(() -> !follower.isBusy()),
                new InstantCommand(() -> {
                    follower.constants.holdPointHeadingScaling = 0.35; // restore for normal hold
                }),
                new RunCommand(() -> {})
        );
    }

    private Command followPathOrUntilIntakeFull(PathChain path, boolean holdEnd) {
        return new ParallelRaceGroup(
                new FollowPathCommand(follower, path, holdEnd),
                new WaitUntilCommand(() -> robot.intake.isStalled())
        );
    }

    @Override
    public void initialize() {
        super.reset();

        alliance = Alliance.BLUE;

        telemetry.addData("Robot Status", "Initializing Auto Blue 12 Far" + alliance.name());
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
        this.paths = new BlueFarFromGoalConstants(follower, robot);

        // Initialize follower
        follower.setStartingPose(paths.blueFarWallStart.withHeading(Math.toRadians(paths.blueFarWallStartHeading)));
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
                                new FollowPathCommand(follower, paths.blueWallToFarZonePrescore, 0.5).beforeStarting(prepareShot()),
                                new ParallelRaceGroup(
                                        new WaitCommand(1200),
                                        new WaitUntilCommand(() -> robot.shotController.readyToShoot()),
                                        turnToGoal()
                                ),
                                shoot(),
                                new ParallelRaceGroup(
                                        new WaitCommand(TimingConstants.waitForShootingMS),
                                        turnToGoal()
                                ).andThen(idleState()),

                                /**
                                 SECOND CYCLE - intake FAR
                                 **/

                                // Intake from row closest to far wall from goal -- intake starts after 25% of path complete
                                //new FollowPathCommand(follower, paths.blueFarZonePrescoreToFarIntake, false),
                                followPathOrUntilIntakeFull(paths.blueFarZonePrescoreToFarIntake, false),

                                // Score -- preparing shot after 50% of path complete. Begin shooting after path complete
                                new FollowPathCommand(follower, paths.blueFarWallIntakeToFarScore),
                                new ParallelRaceGroup(
                                        new WaitCommand(TimingConstants.waitForShootingMS),
                                        new WaitUntilCommand(() -> robot.shotController.readyToShoot()),
                                        turnToGoal()
                                ),
                                shoot(),
                                new ParallelRaceGroup(
                                        new WaitCommand(TimingConstants.waitForShootingMS),
                                        turnToGoal()
                                ).andThen(idleState()),

                                /**
                                 THIRD CYCLE - intake MID
                                 **/

                                // Intake from row closest to far wall from goal -- intake starts after 25% of path complete
                                //new FollowPathCommand(follower, paths.blueFarZoneScoreToMidIntake, false),
                                followPathOrUntilIntakeFull(paths.blueFarZoneScoreToMidIntake, false),

                                // Score -- preparing shot after 50% of path complete. Begin shooting after path complete
                                new FollowPathCommand(follower, paths.blueMidWallIntakeToFarScore),
                                new ParallelRaceGroup(
                                        new WaitCommand(TimingConstants.waitForShootingMS),
                                        new WaitUntilCommand(() -> robot.shotController.readyToShoot()),
                                        turnToGoal()
                                ),
                                shoot(),
                                new ParallelRaceGroup(
                                        new WaitCommand(TimingConstants.waitForShootingMS),
                                        turnToGoal()
                                ).andThen(idleState()),

                                /**
                                 FOURTH CYCLE - intake Loading Station
                                 **/

                                // Intake from loading station -- intake starts after 25% of path complete
                                //new FollowPathCommand(follower, paths.blueFarWallToIntakeLoadingStation, false),
                                followPathOrUntilIntakeFull(paths.blueFarWallToIntakeLoadingStation, false),
                                //new WaitCommand(TimingConstants.waitForIntakeLoadingStationMS),

                                // Score -- preparing shot after 30% of path complete. Begin shooting after path complete
                                setAutoTargetShot(),
                                new FollowPathCommand(follower, paths.blueFarLoadingStationToFarScore),
                                new ParallelRaceGroup(
                                        new WaitCommand(TimingConstants.waitForShootingMS),
                                        new WaitUntilCommand(() -> robot.shotController.readyToShoot()),
                                        turnToGoal()
                                ),
                                shoot(),
                                new ParallelRaceGroup(
                                        new WaitCommand(TimingConstants.waitForShootingMS),
                                        turnToGoal()
                                ).andThen(intake()),

                                /**
                                 FIFTH CYCLE - intake Loading Station and Park
                                 **/

                                // Intake from loading station -- intake starts after 25% of path complete
                                //new FollowPathCommand(follower, paths.blueFarWallToIntakeLoadingStation, false),
                                followPathOrUntilIntakeFull(paths.blueFarWallToIntakeLoadingStation, false),
                                //new WaitCommand(TimingConstants.waitForIntakeLoadingStationMS),

                                // Score -- preparing shot after 30% of path complete. Begin shooting after path complete
                                setAutoTargetShot(),
                                new FollowPathCommand(follower, paths.blueFarLoadingStationToFarScore),
                                new ParallelRaceGroup(
                                        new WaitCommand(TimingConstants.waitForShootingMS),
                                        new WaitUntilCommand(() -> robot.shotController.readyToShoot()),
                                        turnToGoal()
                                ),
                                shoot(),
                                new ParallelRaceGroup(
                                        new WaitCommand(TimingConstants.waitForShootingMS),
                                        turnToGoal()
                                ).andThen(intake()),

                                // Park
                                new FollowPathCommand(follower, paths.blueFarScoreToPark, false).andThen(spinDown())
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
        resetRuntime();
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
        PanelsUtils.Drawing.drawPath(paths.blueWallToFarZonePrescore);
        PanelsUtils.Drawing.drawPath(paths.blueFarZonePrescoreToFarIntake);
        PanelsUtils.Drawing.drawPath(paths.blueFarWallIntakeToFarScore);
        PanelsUtils.Drawing.drawPath(paths.blueFarZoneScoreToMidIntake);
        PanelsUtils.Drawing.drawPath(paths.blueMidWallIntakeToFarScore);
        PanelsUtils.Drawing.drawPath(paths.blueFarWallToIntakeLoadingStation);
    }
}
