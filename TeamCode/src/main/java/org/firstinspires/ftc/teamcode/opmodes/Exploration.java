package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.constants.Alliance;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.constants.OdometryConstants;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.lib.triggers.ControllerTriggers;
import org.firstinspires.ftc.teamcode.lib.util.BlackboardHelpers;
import org.firstinspires.ftc.teamcode.lib.util.NovaPyraUtils;
import org.firstinspires.ftc.teamcode.lib.panels.PanelsUtils.Drawing;
import org.firstinspires.ftc.teamcode.triggers.CompetitionTriggers;
import org.firstinspires.ftc.teamcode.triggers.RobotTriggers;
import org.firstinspires.ftc.teamcode.triggers.TuningTriggers;

/**
 *   Uses Pedro's PID coefficients with proper scaling
 * - Uses same tuning as autonomous (FollowerConstants)
 * - Applies holdPointHeadingScaling like Pedro does in holdPoint()
 * - Maintains PID state while auto-aiming
 * - Single place to tune heading PID
 */
public class Exploration extends CommandOpMode {
    private NovaPyraRobot robot;
    public Alliance alliance;
    public GamepadEx controller1;
    public GamepadEx controller2;
    private ControllerTriggers controllerMappings;
    private RobotTriggers robotTriggers;

    // Using Pedro's PIDFController
    private PIDFController headingController;
    private PIDFController secondaryHeadingController;

    // Track auto-aim state to maintain PID state properly
    private boolean wasAutoAiming = false;

    public Exploration(Alliance allianceSetting) {
        super();
        alliance = allianceSetting;
    }

    @Override
    public void initialize() {
        super.reset();

        // Setup Dashboard if enabled
        if (Constants.DebugModes.ENABLE_PANELS_DASHBOARD) {
            telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
            Drawing.init();
            Drawing.sendPacket();
        }

        robot = new NovaPyraRobot(hardwareMap, telemetry, alliance);
        robotTriggers = new RobotTriggers(robot);

        // Controllers
        controller1 = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);

        // Controller Mappings
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

        Pose startingPose = BlackboardHelpers.getAutoEndPoseOrDefault(blackboard, alliance);
        robot.pedroFollower.setPose(startingPose);
        robot.pedroFollower.startTeleopDrive();

        // Initialize heading controllers with Pedro's coefficients
        initializeHeadingControllers();

        telemetry.addData("Robot Status", "Initialized Exploration " + alliance.name());
        telemetry.update();
    }

    /**
     * Initialize heading controllers using Pedro's PID coefficients.
     * This ensures consistent tuning between autonomous and teleop.
     */
    private void initializeHeadingControllers() {
        FollowerConstants constants = robot.pedroFollower.getConstants();

        // Primary heading controller (for large errors)
        PIDFCoefficients primaryCoeffs = constants.coefficientsHeadingPIDF;
        headingController = new PIDFController(primaryCoeffs);

        // Secondary heading controller (for fine tuning)
        PIDFCoefficients secondaryCoeffs = constants.coefficientsSecondaryHeadingPIDF;
        secondaryHeadingController = new PIDFController(secondaryCoeffs);

        // Optional: Debug telemetry
        if (Constants.DebugModes.Telemetry.DEBUG_AUTO_AIM) {
            telemetry.addLine("=== Heading PID Coefficients ===");
            telemetry.addData("Primary kP", primaryCoeffs.P);
            telemetry.addData("Primary kD", primaryCoeffs.D);
            telemetry.addData("Primary kF", primaryCoeffs.F);
            telemetry.addData("Secondary kP", secondaryCoeffs.P);
            telemetry.addData("Secondary kD", secondaryCoeffs.D);
            telemetry.addData("Secondary kF", secondaryCoeffs.F);
            telemetry.addData("Heading Scaling", constants.holdPointHeadingScaling);
            telemetry.update();
        }
    }

    @Override
    public void initialize_loop() {
        super.initialize_loop();
        if (Constants.DebugModes.ENABLE_PANELS_DASHBOARD) {
            Drawing.drawVisionAndCurrent(robot.vision, robot.pedroFollower);
        }
    }

    @Override
    public void run() {
        super.run();

        robot.trackCycleTime(getRuntime());

        // Get driver translation inputs
        double[] driveAxes = NovaPyraUtils.deadbandAndSmooth(
                controller1.getLeftY(),
                -controller1.getLeftX(),
                DriveConstants.Joysticks.driveSmoothFactor,
                DriveConstants.Joysticks.driveDeadband,
                DriveConstants.Joysticks.driveInverseDeadband
        );

        // Get driver rotation input
        double rotateAxis = NovaPyraUtils.deadbandAndSmoothValue(
                -controller1.getRightX(),
                DriveConstants.Joysticks.rotateSmoothFactor,
                DriveConstants.Joysticks.rotateDeadband,
                DriveConstants.Joysticks.rotateInverseDeadband
        );

        // ============================================================================
        // AUTO-AIM LOGIC (Limelight with odometry backup)
        // ============================================================================

        boolean useAutoAim = robot.shotController.doAutoHeading() ||
                controller1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON);

       /*
        // Reset controllers when transitioning INTO auto-aim mode
        // This gives a fresh start but maintains state WHILE auto-aiming
        if (useAutoAim && !wasAutoAiming) {
            headingController.reset();
            secondaryHeadingController.reset();
        }

        */

        if (useAutoAim) {
            // Calculate target heading
            double targetHeading = calculateAutoAimTargetHeading();
            double currentHeading = robot.pedroFollower.getPose().getHeading();
            double headingError = normalizeAngle(targetHeading - currentHeading);

            // Use Pedro's PID with proper scaling (like holdPoint does)
            rotateAxis = calculateRotationPower(headingError);

        } else {
            // Manual control
            rotateAxis /= DriveConstants.Joysticks.rotateScalingFactor;
        }

        // Update state tracking
        wasAutoAiming = useAutoAim;

        // ============================================================================
        // Drive (field-centric for both auto-aim and manual)
        // ============================================================================

        robot.pedroFollower.setTeleOpDrive(
                driveAxes[0],
                driveAxes[1],
                rotateAxis,
                false,  // Field-centric
                Math.toRadians(OdometryConstants.DefaultStartPose.allianceDriveHeadingOffset(alliance))
        );
        robot.pedroFollower.update();

        // Update odometry with vision measurements
        if (robot.vision.isPoseValid()) {
            robot.pedroFollower.addVisionUpdate(
                    robot.vision.getPedroPose(),
                    System.nanoTime() - robot.vision.getVisionLatencyNS()
            );
        }

        robot.shotController.updateState();

        // ============================================================================
        // Telemetry
        // ============================================================================

        if (Constants.DebugModes.Telemetry.DEBUG_AUTO_AIM) {
            addAutoAimDebugTelemetry();
        }

        if (Constants.DebugModes.ENABLE_PANELS_DASHBOARD) {
            Drawing.drawVisionAndCurrentAndHistory(robot.vision, robot.pedroFollower);
            Drawing.sendPacket();
        }

        telemetry.update();
    }

    @Override
    public void end() {
        robot.vision.stopVision();
        BlackboardHelpers.clearAutoEndPose(blackboard);
        telemetry.addData("Robot Status", "Stopped Exploration " + alliance.name());
    }

    @Override
    public void reset() {
        robot.reset();
    }

    // ============================================================================
    // AUTO-AIM HELPER METHODS
    // ============================================================================

    /**
     * Calculate the target heading (in radians) that the robot should face
     * to aim at the goal. Uses Limelight if available, odometry as fallback.
     *
     * @return Target heading in radians (field-centric)
     */
    private double calculateAutoAimTargetHeading() {
        Pose currentPose = robot.pedroFollower.getPose();

        // Priority 1: Use Limelight if seeing target
        if (robot.vision.isSeeingTarget()) {
            // Limelight gives angle error from current heading
            double angleErrorDeg = robot.vision.angleToTarget();
            double angleErrorRad = Math.toRadians(angleErrorDeg);
            return currentPose.getHeading() + angleErrorRad;
        }

        // Priority 2: Use odometry if vision pose estimate is enabled
        if (DriveConstants.PoseEstimator.useVisionPoseEstimate) {
            Pose targetPose = robot.vision.getTargetTagPose();

            // Calculate bearing from robot to target
            double dx = targetPose.getX() - currentPose.getX();
            double dy = targetPose.getY() - currentPose.getY();
            return Math.atan2(dy, dx);
        }

        // Fallback: maintain current heading (no auto-aim)
        return currentPose.getHeading();
    }

    /**
     * Calculate rotation power from heading error using Pedro's PID.
     * This exactly mimics what Pedro does in holdPoint() mode.
     * Includes proper scaling factor to prevent oscillation.
     *
     * @param headingError Error in radians (target - current)
     * @return Rotation power [-1.0, 1.0]
     */
    private double calculateRotationPower(double headingError) {
        FollowerConstants constants = robot.pedroFollower.getConstants();
        double headingPIDFSwitch = constants.headingPIDFSwitch;
        boolean useSecondary = constants.useSecondaryHeadingPIDF;

        // CRITICAL: Apply the same scaling Pedro uses in holdPoint()
        double holdPointHeadingScaling = DriveConstants.HeadingControl.holdPoint;

        double output;
        Pose currentPose = robot.pedroFollower.getPose();
        double targetHeading = currentPose.getHeading() + headingError;

        // Switch between primary and secondary PID based on error magnitude
        if (Math.abs(headingError) < headingPIDFSwitch && useSecondary) {
            // Small error - use secondary PID (fine tuning)
            secondaryHeadingController.updateFeedForwardInput(
                    MathFunctions.getTurnDirection(currentPose.getHeading(), targetHeading)
            );
            secondaryHeadingController.updateError(headingError);
            output = secondaryHeadingController.run();

        } else {
            // Large error - use primary PID
            headingController.updateFeedForwardInput(
                    MathFunctions.getTurnDirection(currentPose.getHeading(), targetHeading)
            );
            headingController.updateError(headingError);
            output = headingController.run();
        }

        // Apply scaling (this is what prevents oscillation!)
        // Pedro does: getHeadingVector().times(holdPointHeadingScaling)
        output *= holdPointHeadingScaling;

        return MathFunctions.clamp(output, -1.0, 1.0);
    }

    /**
     * Normalize angle to (-π, π] range
     */
    private double normalizeAngle(double angleRad) {
        while (angleRad > Math.PI) angleRad -= 2.0 * Math.PI;
        while (angleRad <= -Math.PI) angleRad += 2.0 * Math.PI;
        return angleRad;
    }

    /**
     * Check if robot is aligned with target within tolerance
     */
    public boolean isAutoAimAligned() {
        double targetHeading = calculateAutoAimTargetHeading();
        double currentHeading = robot.pedroFollower.getPose().getHeading();
        double headingError = normalizeAngle(targetHeading - currentHeading);
        double headingErrorDeg = Math.toDegrees(Math.abs(headingError));

        return headingErrorDeg <= VisionConstants.headingAngleThresholdDeg;
    }

    /**
     * Debug telemetry for auto-aim system
     */
    private void addAutoAimDebugTelemetry() {
        Pose currentPose = robot.pedroFollower.getPose();
        Pose targetPose = robot.vision.getTargetTagPose();
        double targetHeading = calculateAutoAimTargetHeading();
        double headingError = normalizeAngle(targetHeading - currentPose.getHeading());

        telemetry.addLine("═══ AUTO-AIM DEBUG ═══");

        // Robot state
        telemetry.addData("Robot Pose", "(%.1f, %.1f) @ %.1f°",
                currentPose.getX(),
                currentPose.getY(),
                Math.toDegrees(currentPose.getHeading())
        );

        // Target state
        telemetry.addData("Target Pose", "(%.1f, %.1f)",
                targetPose.getX(),
                targetPose.getY()
        );

        // Vision status
        telemetry.addData("Vision Active", robot.vision.isSeeingTarget());
        if (robot.vision.isSeeingTarget()) {
            telemetry.addData("  LL Angle", "%.1f°", robot.vision.angleToTarget());
            telemetry.addData("  LL Distance", "%.1f cm", robot.vision.distanceToTarget());
        }

        // Auto-aim calculation
        telemetry.addData("Target Heading", "%.1f°", Math.toDegrees(targetHeading));
        telemetry.addData("Heading Error", "%.1f°", Math.toDegrees(headingError));
        telemetry.addData("Aligned", isAutoAimAligned() ? "✓ YES" : "✗ NO");
        telemetry.addData("Auto-Aim Active", wasAutoAiming ? "YES" : "NO");

        // Controller state
        FollowerConstants constants = robot.pedroFollower.getConstants();
        boolean usingSecondary = Math.abs(headingError) < constants.headingPIDFSwitch;
        telemetry.addData("PID Controller", usingSecondary ? "Secondary (fine)" : "Primary (coarse)");
        telemetry.addData("Scaling Factor", "%.2f", constants.holdPointHeadingScaling);
    }
}