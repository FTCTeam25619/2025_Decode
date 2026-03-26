package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.autos.AutoPositionTunerConstants;
import org.firstinspires.ftc.teamcode.constants.Alliance;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.lib.orchestration.ShotCycleState;
import org.firstinspires.ftc.teamcode.lib.util.BlackboardHelpers;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.triggers.RobotTriggers;

@Autonomous(name = "Position Tuner", group = "Autonomous")
@Configurable
public class AutoPositionTuner extends CommandOpMode {
    private NovaPyraRobot robot;
    private RobotTriggers robotTriggers;
    private Follower follower;
    private AutoPositionTunerConstants paths;

    protected TelemetryData telemetryData;
    private Alliance alliance;

    private InstantCommand spinDown() {
        return new InstantCommand(() -> {
            robot.shotController.setState(ShotCycleState.OFF);
        });
    }

    @Override
    public void initialize() {
        super.reset();

        alliance = Alliance.BLUE;

        telemetry.addData("Robot Status", "Initializing Position Tuner " + alliance.name());
        telemetry.update();

        if (Constants.DebugModes.ENABLE_PANELS_DASHBOARD) {
            telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        }

        this.telemetryData = new TelemetryData(telemetry);

        this.robot = new NovaPyraRobot(hardwareMap, telemetry, alliance);
        this.robotTriggers = new RobotTriggers(robot);
        this.robotTriggers.setupNonControllerTriggers();

        robot.vision.setTargetingTag(
                alliance == Alliance.BLUE ?
                        VisionConstants.AprilTags.BlueGoal :
                        VisionConstants.AprilTags.RedGoal
        );

        this.follower = robot.pedroFollower;
        this.paths = new AutoPositionTunerConstants(follower, robot);

        // Initialize follower with starting pose
        follower.setStartingPose(paths.startPose.withHeading(Math.toRadians(paths.startHeading)));
        BlackboardHelpers.setAutoEndPose(blackboard, follower.getPose());


        schedule(
                new SequentialCommandGroup(
                        spinDown(),
                        // Hold for 5 minutes to allow manual positioning
                        new WaitCommand(300000))
        );

        telemetry.addData("Robot Status", "Position Tuner Ready — push robot to map points");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        // Must call update() each tick so Pedro localizes the pushed position
        follower.update();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetryData.update();
    }

    @Override
    public void end() {
        super.end();
        BlackboardHelpers.setAutoEndPose(blackboard, follower.getPose());
    }
}