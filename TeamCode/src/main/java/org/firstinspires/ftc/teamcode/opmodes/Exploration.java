package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
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
 * IMPROVED AUTO-AIM - HEADINGTUNER APPROACH
 *
 * Key insight: HeadingTuner follows a path with only heading PID active.
 * We do the same but create a "virtual path" that moves with the robot.
 *
 * This approach:
 * 1. Creates a Path with constant heading interpolation (like HeadingTuner)
 * 2. Disables drive and translational PID (only heading active)
 * 3. Lets driver control translation via teleop
 * 4. Pedro's heading PID runs automatically via followPath()
 *
 * Benefits:
 * - Exact same behavior as HeadingTuner
 * - Driver can translate freely
 * - Full PID power (no damping)
 * - Single tuning point (HeadingTuner)
 */
public class Exploration extends CommandOpMode {
    private NovaPyraRobot robot;
    public Alliance alliance;
    public GamepadEx controller1;
    public GamepadEx controller2;
    private ControllerTriggers controllerMappings;
    private RobotTriggers robotTriggers;

    // Track auto-aim state
    private boolean wasAutoAiming = false;
    private Path autoAimPath = null;
    private double lastTargetHeading = 0;

    // How far ahead to create the "virtual" path point
    // Large enough that we never reach it, small enough to update quickly
    private static final double VIRTUAL_PATH_DISTANCE = 100.0;  // inches

    public Exploration(Alliance allianceSetting) {
        super();
        alliance = allianceSetting;
    }

    @Override
    public void initialize() {
        super.reset();

        if (Constants.DebugModes.ENABLE_PANELS_DASHBOARD) {
            telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
            Drawing.init();
            Drawing.sendPacket();
        }

        robot = new NovaPyraRobot(hardwareMap, telemetry, alliance, true);
        robotTriggers = new RobotTriggers(robot);

        controller1 = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);

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

        telemetry.addData("Robot Status", "Initialized Exploration " + alliance.name());
        telemetry.update();
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

        // Get driver rotation input (only used in manual mode)
        double rotateAxis = NovaPyraUtils.deadbandAndSmoothValue(
                -controller1.getRightX(),
                DriveConstants.Joysticks.rotateSmoothFactor,
                DriveConstants.Joysticks.rotateDeadband,
                DriveConstants.Joysticks.rotateInverseDeadband
        );

        // ============================================================================
        // AUTO-AIM LOGIC - HeadingTuner Approach
        // ============================================================================

        boolean useAutoAim = robot.shotController.doAutoHeading() ||
                controller1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON);

        if (useAutoAim) {
            // ═══ AUTO-AIM MODE: Use path following with heading-only PID ═══

            Pose currentPose = robot.pedroFollower.getPose();
            double targetHeading = calculateAutoAimTargetHeading();

            // Check if we need to create/update the path
            boolean needsUpdate = !wasAutoAiming ||
                    autoAimPath == null ||
                    Math.abs(normalizeAngle(targetHeading - lastTargetHeading)) > Math.toRadians(2);

            if (needsUpdate) {
                // Create a "virtual" path that the robot will never reach
                // This mimics what HeadingTuner does
                Pose startPose = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading());
                Pose endPose = new Pose(
                        currentPose.getX() + VIRTUAL_PATH_DISTANCE * Math.cos(currentPose.getHeading()),
                        currentPose.getY() + VIRTUAL_PATH_DISTANCE * Math.sin(currentPose.getHeading()),
                        currentPose.getHeading()
                );

                autoAimPath = new Path(new BezierLine(startPose, endPose));

                // Set constant heading interpolation to target
                // This is exactly what HeadingTuner does!
                autoAimPath.setConstantHeadingInterpolation(targetHeading);

                // Configure PID usage - ONLY heading active (like HeadingTuner)
                robot.pedroFollower.deactivateAllPIDFs();
                robot.pedroFollower.activateHeading();

                // Start following the path
                robot.pedroFollower.followPath(autoAimPath, false);  // false = don't hold at end

                lastTargetHeading = targetHeading;
            }

            // Driver translation will be handled AFTER follower.update()
            // We'll override the drivetrain directly with manual input

        } else {
            // ═══ MANUAL CONTROL MODE: Standard teleop ═══

            // Transition from auto-aim back to teleop
            if (wasAutoAiming) {
                robot.pedroFollower.activateAllPIDFs();  // Re-enable all PIDFs
                robot.pedroFollower.startTeleopDrive();
                autoAimPath = null;
            }

            // Standard field-centric drive
            robot.pedroFollower.setTeleOpDrive(
                    driveAxes[0],
                    driveAxes[1],
                    rotateAxis / DriveConstants.Joysticks.rotateScalingFactor,
                    false,  // Field-centric
                    Math.toRadians(OdometryConstants.DefaultStartPose.allianceDriveHeadingOffset(alliance))
            );
        }

        // ============================================================================
        // Pedro Update FIRST - Computes heading correction from path
        // ============================================================================
        robot.pedroFollower.update();

        // Update odometry with vision measurements
        if (robot.vision.isPoseValid()) {
            robot.pedroFollower.addVisionUpdate(
                    robot.vision.getPedroPose(),
                    System.nanoTime() - robot.vision.getVisionLatencyNS()
            );
        }

        // ============================================================================
        // THEN handle auto-aim override (after Pedro computed heading vector)
        // ============================================================================
        if (useAutoAim) {
            // Pedro's update() already computed heading correction from the path
            // Now we override the drivetrain with manual drive + Pedro's heading

            Pose currentPose = robot.pedroFollower.getPose();
            double headingOffset = Math.toRadians(
                    OdometryConstants.DefaultStartPose.allianceDriveHeadingOffset(alliance)
            );

            // Create drive vector from joystick
            // When using field coordinates, use axes as-is (not swapped)
            Vector driveVector = new Vector();
            driveVector.setOrthogonalComponents(driveAxes[0], driveAxes[1]);


            // Scale to max power
            double maxPowerScaling = robot.pedroFollower.getMaxPowerScaling();
            driveVector.setMagnitude(driveVector.getMagnitude() * maxPowerScaling);

            // For field-centric control:
            // - Blue alliance (offset=0°): Rotate ONLY by headingOffset (which is 0, so no rotation)
            //   This keeps the vector in absolute field coordinates
            // - Red alliance (offset=180°): Rotate by headingOffset (180°)
            //   This flips the vector for the other side of the field
            //
            // The key insight: We DON'T rotate by currentHeading here!
            // The drivetrain will handle converting from field to robot coordinates
            driveVector.rotateVector(headingOffset);


            // Get Pedro's heading correction
            Vector headingVector = robot.pedroFollower.getHeadingVector();

            if (Constants.DebugModes.Telemetry.DEBUG_AUTO_AIM) {
                telemetry.addData("Heading Vector", "Mag: %.3f", headingVector.getMagnitude());
            }

            // Pass the field coordinate vector (rotated only by headingOffset)
            // The drivetrain's calculateDrive() will handle field-to-robot conversion
            robot.pedroFollower.getDrivetrain().runDrive(
                    new Vector(),          // No corrective vector
                    headingVector,         // Pedro's heading from path
                    driveVector,           // Field coordinates (offset-rotated only)
                    currentPose.getHeading()
            );
        }
        // If not auto-aiming, follower.update() already handled everything

        // Update state tracking
        wasAutoAiming = useAutoAim;

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

       // Use odometry if vision pose estimate is enabled
        if (DriveConstants.PoseEstimator.useVisionPoseEstimate) {
            Pose targetPose = robot.vision.getTargetTagPose();
            double dx = targetPose.getX() - currentPose.getX();
            double dy = targetPose.getY() - currentPose.getY();
            return Math.atan2(dy, dx);
        }

        // Fallback: maintain current heading
        return currentPose.getHeading();
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
        if (!wasAutoAiming) return false;

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

        telemetry.addLine("═══ AUTO-AIM DEBUG (HeadingTuner Method) ═══");

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
        telemetry.addData("  Odom Distance", "%.1f cm", robot.shotController.getDistanceToTarget());
        telemetry.addData("Target Heading", "%.1f°", Math.toDegrees(targetHeading));
        telemetry.addData("Heading Error", "%.1f°", Math.toDegrees(headingError));
        telemetry.addData("Aligned", isAutoAimAligned() ? "✓ YES" : "✗ NO");


    }
}