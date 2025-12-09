
package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.constants.OpModes;
import org.firstinspires.ftc.teamcode.constants.OpModes.OpModeSelection;
import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.lib.triggers.ControllerTriggers;
import org.firstinspires.ftc.teamcode.lib.util.NovaPyraUtils;
import org.firstinspires.ftc.teamcode.triggers.CompetitionTriggers;
import org.firstinspires.ftc.teamcode.triggers.RobotTriggers;
import org.firstinspires.ftc.teamcode.triggers.TuningTriggers;

public class Exploration extends CommandOpMode {
    private NovaPyraRobot robot;
    public OpModes.Alliance alliance;
    public GamepadEx controller1;
    public GamepadEx controller2;
    private ControllerTriggers controllerMappings;
    private RobotTriggers robotTriggers;

    public Exploration(OpModes.Alliance allianceSetting) {
        super();
        alliance = allianceSetting;
    }

    @Override
    public void initialize() {
        super.reset();

        // Setup Dashboard if setup for shop debugging
        if (Constants.DebugModes.ENABLE_PANELS_DASHBOARD) {
            telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        }

        robot = new NovaPyraRobot(hardwareMap, telemetry, alliance);
        robotTriggers = new RobotTriggers(robot);


        // Controllers
        controller1 = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);

        // ControllerMappings
        if (Constants.DebugModes.ControlInterface.TUNING_CONTROLS) {
            controllerMappings = new TuningTriggers(robot, controller1, controller2);
        } else {
            controllerMappings = new CompetitionTriggers(robot, controller1, controller2);
        }

        robotTriggers.setupNonControllerTriggers();
        controllerMappings.setupGamepadButtonMappings();
        robot.vision.startVision();
        robot.vision.setTargetingTag(alliance == OpModes.Alliance.BLUE ?
                VisionConstants.AprilTags.BlueGoal : VisionConstants.AprilTags.RedGoal);
        robot.odometry.setRobotPose(new Pose2D(DistanceUnit.MM, 0.0, 0.0,
                AngleUnit.DEGREES, robot.allianceDriveHeadingOffset()));

        robot.pedroFollower.startTeleopDrive();

        telemetry.addData("Robot Status", "Initialized Exploration " + alliance.name());
        telemetry.update();
    }

    @Override
    public void initialize_loop() {
        super.initialize_loop();
        // Things that need to run while disabled every cycle
    }

    @Override
    public void run() {
        super.run();

        robot.trackCycleTime(getRuntime());

        double[] driveAxes = NovaPyraUtils.deadbandAndSmooth(
                controller1.getLeftY(),
                -controller1.getLeftX(),
                DriveConstants.Joysticks.driveSmoothFactor,
                DriveConstants.Joysticks.driveDeadband,
                DriveConstants.Joysticks.driveInverseDeadband
        );

        double rotateAxis = NovaPyraUtils.deadbandAndSmoothValue(
                -controller1.getRightX(),
                DriveConstants.Joysticks.rotateSmoothFactor,
                DriveConstants.Joysticks.rotateDeadband,
                DriveConstants.Joysticks.rotateInverseDeadband
        );

        if (controller1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON) && robot.vision.isSeeingTarget()) {
            rotateAxis = MathUtils.clamp(
                    -DriveConstants.HeadingControl.kPHeadingCorrection * robot.vision.angleToTarget(),
                    -DriveConstants.HeadingControl.correctionOutputClamp,
                    DriveConstants.HeadingControl.correctionOutputClamp);
        } else {
            rotateAxis /= DriveConstants.Joysticks.rotateScalingFactor;
        }
        robot.pedroFollower.setTeleOpDrive(driveAxes[0], driveAxes[1], rotateAxis,
                false, Math.toRadians(robot.allianceDriveHeadingOffset()));
        robot.pedroFollower.update();
        robot.shotController.updateState();

        telemetry.addData("X", robot.pedroFollower.getPose().getX());
        telemetry.addData("Y", robot.pedroFollower.getPose().getY());
        telemetry.addData("Heading", robot.pedroFollower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void end() {
        robot.vision.stopVision();
        telemetry.addData("Robot Status", "Stopped Exploration " + alliance.name());
    }

    @Override
    public void reset() {
        robot.reset();
    }
}
