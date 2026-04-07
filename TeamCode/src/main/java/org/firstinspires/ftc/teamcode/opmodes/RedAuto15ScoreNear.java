package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
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
import com.seattlesolvers.solverslib.pedroCommand.HoldPointCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

@Autonomous(name = "Red Near 15", group = "Autonomous")
@Configurable // Panels
public class RedAuto15ScoreNear extends CommandOpMode {
    private NovaPyraRobot robot;
    private RobotTriggers robotTriggers;
    private Follower follower;
    private RedNearGoalConstants paths;
    protected TelemetryData telemetryData;
    private Alliance alliance;

    private InstantCommand setupNearWallPresetShot() {
        return new InstantCommand(() -> {
            robot.shotController.setupPresetShot(PresetShot.NearZoneMid);
        });
    }

    private InstantCommand setupNearMidPresetShot() {
        return new InstantCommand(() -> {
            robot.shotController.setupPresetShot(PresetShot.NearZoneMid);
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
                    follower.constants.holdPointHeadingScaling = .60;
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

        telemetry.addData("Robot Status", "Initializing Auto 15 Ball Near " + alliance.name());
        telemetry.update();

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
        this.paths = new RedNearGoalConstants(follower, robot);

        // Initialize follower
        follower.setStartingPose(paths.redNearWallStart.withHeading(Math.toRadians(paths.redNearWallStartHeading)));
        BlackboardHelpers.setAutoEndPose(blackboard, follower.getPose());

        // Schedule the autonomous sequence
        schedule(
                new ParallelCommandGroup(
                        new RunCommand(follower::update),
                        new SequentialCommandGroup(

                                /**
                                 FIRST CYCLE - Preload
                                 **/

                                new FollowPathCommand(follower, paths.redWallToNearZonePrescore, false).beforeStarting(prepareShot()),
                                idleState(),

                                /**
                                 SECOND CYCLE - MID row
                                 **/

                                followPathOrUntilIntakeFull(paths.redNearZoneScoreToMidIntake, false),

                                setupNearMidPresetShot(),
                                new FollowPathCommand(follower, paths.redMidWallIntakeToNearScore),
                                new WaitCommand(TimingConstants.waitForShootingNearMS).andThen(idleState()),


                                /**
                                 THIRD CYCLE - Gate Open and Intake
                                 **/

                                new FollowPathCommand(follower, paths.redOpenAndIntakeGate).beforeStarting(intake()),

                                new WaitCommand(1000),

                                new FollowPathCommand(follower, paths.redMidWallIntakeToNearScore),
                                new WaitCommand(TimingConstants.waitForShootingNearMS).andThen(idleState()),


                                /**
                                 FOURTH CYCLE - Gate Open and Intake
                                 **/

                                new FollowPathCommand(follower, paths.redOpenAndIntakeGate).beforeStarting(intake()),

                                new WaitCommand(1500),

                                new FollowPathCommand(follower, paths.redMidWallIntakeToNearScore),
                                new WaitCommand(TimingConstants.waitForShootingNearMS).andThen(idleState()),


                                /**
                                 FIFTH CYCLE - row NEAR goal
                                 **/

                                // turn in place facing intake toward near row artifacts
                                // this is done to prevent next path colliding with artifacts
                                new ParallelRaceGroup(
                                        new WaitCommand(500),
                                        new WaitUntilCommand(() -> Math.abs(
                                                AngleUnit.normalizeRadians(follower.getPose().getHeading() - Math.toRadians(0))
                                        ) < Math.toRadians(3)),
                                        new RunCommand(() -> {
                                            follower.constants.holdPointHeadingScaling = 0.85;
                                            follower.turnTo(Math.toRadians(0));
                                        })
                                ),
                                new InstantCommand(() -> follower.constants.holdPointHeadingScaling = 0.35),

                                followPathOrUntilIntakeFull(paths.redNearZoneToNearIntake, false),

                                /**
                                 SCORE & PARK
                                 **/

                                new FollowPathCommand(follower, paths.redNearWallIntakeToNearScoreAndPark),
                                new WaitCommand(TimingConstants.waitForShootingNearMS).andThen(spinDown())
                        )
                )
        );

        telemetry.addData("Robot Status", "Initialized Auto 15 Ball Near " + alliance.name());
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