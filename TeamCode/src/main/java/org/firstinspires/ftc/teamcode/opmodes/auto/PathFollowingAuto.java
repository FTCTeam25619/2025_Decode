package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.constants.Alliance;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.lib.orchestration.PresetShot;
import org.firstinspires.ftc.teamcode.lib.orchestration.ShotCycleState;
import org.firstinspires.ftc.teamcode.lib.panels.PanelsUtils;
import org.firstinspires.ftc.teamcode.lib.triggers.SupplierTrigger;
import org.firstinspires.ftc.teamcode.lib.util.BlackboardHelpers;
import org.firstinspires.ftc.teamcode.triggers.RobotTriggers;

public abstract class PathFollowingAuto extends CommandOpMode {
    private NovaPyraRobot robot;
    private RobotTriggers robotTriggers;
    private Follower follower;
    protected TelemetryData telemetryData;
    private Alliance alliance;
    private String autoName;

    public PathFollowingAuto(String autoName, Alliance allianceSetting) {
        super();

        this.alliance = allianceSetting;
        this.autoName = autoName;

        this.robot = new NovaPyraRobot(hardwareMap, telemetry, alliance);
        this.robotTriggers = new RobotTriggers(robot);
        this.robotTriggers.setupNonControllerTriggers();

        this.follower = robot.pedroFollower;
    }

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

    private Trigger readyToShootPreload() {
        return new SupplierTrigger(() -> (!follower.isBusy()) && robot.shotController.readyToShoot());
    }

    @Override
    public void initialize() {
        super.reset();

        telemetry.addData("Robot Status", "Initializing " + autoName + " " + alliance.name());
        telemetry.update();

        // Setup Dashboard if setup for shop debugging
        if (Constants.DebugModes.ENABLE_PANELS_DASHBOARD) {
            telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
            this.telemetryData = new TelemetryData(telemetry);
            PanelsUtils.Drawing.init();
            PanelsUtils.Drawing.sendPacket();
        }

        // Initialize follower
        follower.setStartingPose(getStartingPose());
        BlackboardHelpers.setAutoEndPose(blackboard, follower.getPose());

        // Schedule the autonomous sequence
        schedule(
                new ParallelCommandGroup(
                        new RunCommand(follower::update),
                        autoSequence()
                )
        );

//        readyToShootPreload().whenActive(shoot());

        telemetry.addData("Robot Status", "Initialized Auto 6 Ball Far " + alliance.name());
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

    public abstract Pose getStartingPose();
    public abstract SequentialCommandGroup autoSequence();
    public abstract void drawPaths();
}
