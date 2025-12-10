
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
import org.firstinspires.ftc.teamcode.constants.Alliance;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.lib.triggers.ControllerTriggers;
import org.firstinspires.ftc.teamcode.lib.util.NovaPyraUtils;
import org.firstinspires.ftc.teamcode.triggers.CompetitionTriggers;
import org.firstinspires.ftc.teamcode.triggers.RobotTriggers;
import org.firstinspires.ftc.teamcode.triggers.TuningTriggers;

public class Exploration extends CommandOpMode {
    private NovaPyraRobot robot;
    public Alliance alliance;
    public GamepadEx controller1;
    public GamepadEx controller2;
    private ControllerTriggers controllerMappings;
    private RobotTriggers robotTriggers;

    public Exploration(Alliance allianceSetting) {
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
        robot.vision.setTargetingTag(alliance == Alliance.BLUE ?
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

        // ----------------------------------------------------
        // AUTO-AIM LOGIC (Limelight-only, no odom backup)
        // ----------------------------------------------------

        // Automatic aiming during PREPARE_SHOT / READY_TO_SHOOT / SHOOTING / RECOVERY
        if (robot.shotController.isShooting() && robot.vision.isSeeingTarget()) {
            double headingErrorDeg = robot.vision.angleToTarget();
            rotateAxis = MathUtils.clamp(
                    -DriveConstants.HeadingControl.kPHeadingCorrection * headingErrorDeg,
                    -DriveConstants.HeadingControl.correctionOutputClamp,
                    DriveConstants.HeadingControl.correctionOutputClamp
            );
        }
        // --- AIM ASSIST BUTTON (only when NOT preparing shot or shooting) ---
        else if (controller1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON) && robot.vision.isSeeingTarget()) {
            rotateAxis = MathUtils.clamp(
                    -DriveConstants.HeadingControl.kPHeadingCorrection * robot.vision.angleToTarget(),
                    -DriveConstants.HeadingControl.correctionOutputClamp,
                    DriveConstants.HeadingControl.correctionOutputClamp);
        } else {
            rotateAxis /= DriveConstants.Joysticks.rotateScalingFactor;
        }

        // ----------------------------------------------------
        // Drive, update follower + shot controller, telemetry
        // ----------------------------------------------------
        robot.pedroFollower.setTeleOpDrive(driveAxes[0], driveAxes[1], rotateAxis,
                false, Math.toRadians(robot.allianceDriveHeadingOffset()));
        robot.pedroFollower.update();
        robot.shotController.updateState();

        telemetry.addData("X", robot.pedroFollower.getPose().getX());
        telemetry.addData("Y", robot.pedroFollower.getPose().getY());
        telemetry.addData("Heading", robot.pedroFollower.getPose().getHeading());
        telemetry.addData("LL Seeing Target", robot.vision.isSeeingTarget());
        telemetry.addData("LL Angle (deg)", robot.vision.angleToTarget());
        telemetry.addData("Is Shooting?", robot.shotController.isShooting());
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

    // --- AUTO AIM HELPERS (Limelight-only for now) ---

    /**
     * One step of auto-aim:
     * - If Limelight sees the tag, use tx (angleToTarget) to rotate.
     * - If no target, do nothing (no odometry backup for now).
     * This ONLY sets teleOpDrive rotation; follower.update() is called elsewhere.
     */
    public void autoAimStep() {
        double rotateAxis = 0.0;

        if (robot.vision.isSeeingTarget()) {
            double headingErrorDeg = robot.vision.angleToTarget();
            rotateAxis = MathUtils.clamp(
                    -DriveConstants.HeadingControl.kPHeadingCorrection * headingErrorDeg,
                    -DriveConstants.HeadingControl.correctionOutputClamp,
                    DriveConstants.HeadingControl.correctionOutputClamp
            );
        }

        // No translation while lining up – just rotate (if rotateAxis == 0, this is a no-op)
        robot.pedroFollower.setTeleOpDrive(0.0, 0.0, rotateAxis, false);
    }

    /**
     * Returns true when we're "close enough" to facing the target.
     * For now, ONLY uses Limelight error and tolerance.
     */
    public boolean isAutoAimAligned() {
        if (!robot.vision.isSeeingTarget()) {
            return false; // no target, not aligned
        }
        return Math.abs(robot.vision.angleToTarget()) <= VisionConstants.headingAngleThresholdDeg;
    }
}
