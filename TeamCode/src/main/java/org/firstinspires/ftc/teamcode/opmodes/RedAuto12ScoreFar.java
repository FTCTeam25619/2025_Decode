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

@Autonomous(name = "Red Far 15", group = "Autonomous")
@Configurable // Panels
public class RedAuto12ScoreFar extends CommandOpMode {
    private NovaPyraRobot robot;
    private RobotTriggers robotTriggers;
    private Follower follower;
    private RedFarFromGoalConstants paths;
    protected TelemetryData telemetryData;
    private Alliance alliance;

    private static boolean firstRun = true;

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
                    follower.constants.holdPointHeadingScaling = 0.85;
                    follower.turnTo(Math.atan2(dy, dx));
                }),
                new WaitUntilCommand(() -> !follower.isBusy()),
                new InstantCommand(() -> {
                    follower.constants.holdPointHeadingScaling = 0.35;
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

        alliance = Alliance.RED;

        telemetry.addData("Robot Status", "Initializing Auto Red 15 Far + LS " + alliance.name());
        telemetry.update();

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

                                setupFarWallPresetShot(),
                                new FollowPathCommand(follower, paths.redWallToFarZonePrescore, 0.5).beforeStarting(prepareShot()),
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

                                followPathOrUntilIntakeFull(paths.redFarZonePrescoreToFarIntake, false),

                                new FollowPathCommand(follower, paths.redFarWallIntakeToFarScore),
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

                                followPathOrUntilIntakeFull(paths.redFarZoneScoreToMidIntake, false),

                                new FollowPathCommand(follower, paths.redMidWallIntakeToFarScore),
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

                                followPathOrUntilIntakeFull(paths.redFarWallToIntakeLoadingStation, false),

                                setAutoTargetShot(),
                                new FollowPathCommand(follower, paths.redFarLoadingStationToFarScore),
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

                                followPathOrUntilIntakeFull(paths.redFarWallToIntakeLoadingStation, false),

                                setAutoTargetShot(),
                                new FollowPathCommand(follower, paths.redFarLoadingStationToFarScore),
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
                                new FollowPathCommand(follower, paths.redFarScoreToPark, false).andThen(spinDown())
                        )
                )
        );

        telemetry.addData("Robot Status", "Initialized Auto 15 Ball Far + LS " + alliance.name());
        telemetry.update();
    }

    public void initialize_loop() {
        super.initialize_loop();
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
        PanelsUtils.Drawing.drawPath(paths.redWallToFarZonePrescore);
        PanelsUtils.Drawing.drawPath(paths.redFarZonePrescoreToFarIntake);
        PanelsUtils.Drawing.drawPath(paths.redFarWallIntakeToFarScore);
        PanelsUtils.Drawing.drawPath(paths.redFarZoneScoreToMidIntake);
        PanelsUtils.Drawing.drawPath(paths.redMidWallIntakeToFarScore);
        PanelsUtils.Drawing.drawPath(paths.redFarWallToIntakeLoadingStation);
    }
}