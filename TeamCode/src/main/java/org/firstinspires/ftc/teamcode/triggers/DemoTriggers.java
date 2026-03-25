package org.firstinspires.ftc.teamcode.triggers;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.constants.HoodConstants;
import org.firstinspires.ftc.teamcode.constants.OdometryConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterFeedConstants;
import org.firstinspires.ftc.teamcode.lib.orchestration.ShotCycleState;
import org.firstinspires.ftc.teamcode.lib.triggers.ControllerTriggers;
import org.firstinspires.ftc.teamcode.lib.triggers.TriggerAxis;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeed;

/**
 * DemoTriggers
 *
 * Shooting bypasses ShotController entirely — subsystems are commanded directly.
 * ShotController is only used for INTAKE state.
 *
 * Controller 1 (Driver):
 *   Left Stick       : Drive translation (field-centric)
 *   Right Stick X    : Rotate
 *   Left Trigger     : Intake (via ShotController)
 *   Right Trigger    : Shoot — opens feed gate, runs feed wheel and intake
 *   Start            : Reset odometry heading
 *
 * Controller 2 (Operator):
 *   Y Button         : Far preset   — 2900 RPM
 *   X Button         : Mid preset   — 2300 RPM
 *   A Button         : Close preset — 1900 RPM
 *   Right Trigger    : Hood UP   (while held, rate-based)
 *   Left Trigger     : Hood DOWN (while held, rate-based)
 */
public class DemoTriggers implements ControllerTriggers {

    private static final double HOOD_DEG_PER_SEC      = 120.0;
    private static final double HOOD_TRIGGER_DEADBAND = 0.08;
    // Low-pass smoothing factor (0.0–1.0). Higher = faster response but more jitter.
    // Lower = smoother but slightly laggy. 0.15 is a good starting point.
    private static final double SMOOTH_FACTOR         = 0.15;

    private static final double RPM_FAR   = 2900.0;
    private static final double RPM_MID   = 2300.0;
    private static final double RPM_CLOSE = 1900.0;

    private final NovaPyraRobot robot;
    private final GamepadEx     controller1;
    private final GamepadEx     controller2;

    private double selectedRPM    = RPM_MID;
    private double hoodTargetDeg  = 20.0;  // where the trigger is commanding
    private double hoodCurrentDeg = 20.0;  // smoothed value actually sent to servo
    private long   lastHoodNanos  = 0;

    // Feed gate delay: gate opens immediately on RT press,
    // feed wheel and intake held off for 100 ms to let the gate clear.
    private static final long FEED_DELAY_NS     = 100_000_000L; // 100 ms
    private boolean           shootPending         = false;
    private long              shootGateOpenedNanos = 0;

    public DemoTriggers(NovaPyraRobot robot, GamepadEx controller1, GamepadEx controller2) {
        this.robot       = robot;
        this.controller1 = controller1;
        this.controller2 = controller2;
    }

    @Override
    public void setupGamepadButtonMappings() {
        setupController1ButtonMappings();
        setupController2ButtonMappings();
    }

    // -------------------------------------------------------------------------
    // Controller 1
    // -------------------------------------------------------------------------

    private void setupController1ButtonMappings() {
        TriggerAxis   c1LeftTrigger  = new TriggerAxis(controller1, GamepadKeys.Trigger.LEFT_TRIGGER,  DEFAULT_AXIS_THRESHOLD);
        TriggerAxis   c1RightTrigger = new TriggerAxis(controller1, GamepadKeys.Trigger.RIGHT_TRIGGER, DEFAULT_AXIS_THRESHOLD);
        GamepadButton c1Start        = new GamepadButton(controller1, GamepadKeys.Button.START);

        // LEFT TRIGGER: Intake via ShotController
        c1LeftTrigger.whenActive(new InstantCommand(() ->
                robot.shotController.setState(ShotCycleState.INTAKE)));
        c1LeftTrigger.whenInactive(new InstantCommand(() ->
                robot.shotController.setState(ShotCycleState.IDLE)));

        // RIGHT TRIGGER: gate opens immediately on press; feed wheel and intake
        // are held off until 100 ms has elapsed (see updateShootDelay()).
        c1RightTrigger.whenActive(new InstantCommand(() -> {
            robot.feedStop.disengage();
            shootPending         = true;
            shootGateOpenedNanos = System.nanoTime();
        }));
        c1RightTrigger.whenInactive(new InstantCommand(() -> {
            shootPending = false;
            robot.feedStop.engage();
            robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
            robot.intake.setState(Intake.IntakeState.Stopped);
        }));

        // START: reset odometry heading
        c1Start.whenPressed(new InstantCommand(() ->
                robot.odometry.setOdometryHeading(
                        OdometryConstants.DefaultStartPose.allianceDriveHeadingOffset(robot.alliance),
                        AngleUnit.DEGREES)));
    }

    // -------------------------------------------------------------------------
    // Controller 2
    // -------------------------------------------------------------------------

    private void setupController2ButtonMappings() {
        GamepadButton c2Y = new GamepadButton(controller2, GamepadKeys.Button.Y);
        GamepadButton c2X = new GamepadButton(controller2, GamepadKeys.Button.X);
        GamepadButton c2A = new GamepadButton(controller2, GamepadKeys.Button.A);

        c2Y.whenActive(new InstantCommand(() -> selectedRPM = RPM_FAR));
        c2X.whenActive(new InstantCommand(() -> selectedRPM = RPM_MID));
        c2A.whenActive(new InstantCommand(() -> selectedRPM = RPM_CLOSE));
    }

    // -------------------------------------------------------------------------
    // Per-loop updates
    // -------------------------------------------------------------------------

    /** Keeps the flywheel spinning at the selected RPM every loop. */
    public void updateRPM() {
        robot.shooter.setState(Shooter.ShooterState.Shooting);
        robot.shooter.setTargetRPM(selectedRPM);
    }

    /**
     * Call every loop. Once 100 ms has elapsed since the feed gate was opened,
     * starts the feed wheel and intake. Safe to call even when not shooting.
     */
    public void updateShootDelay() {
        if (!shootPending) return;
        if (System.nanoTime() - shootGateOpenedNanos >= FEED_DELAY_NS) {
            robot.shooterFeed.setFeedPower(ShooterFeedConstants.Targets.defaultFeedPower);
            robot.shooterFeed.setState(ShooterFeed.FeedState.Feeding);
            robot.intake.setState(Intake.IntakeState.Intake);
            shootPending = false; // only need to trigger this once
        }
    }

    /** Moves hood based on C2 trigger depth × elapsed time. */
    public void updateHood() {
        long   now = System.nanoTime();
        double dt  = (lastHoodNanos == 0) ? 0.0 : Math.min((now - lastHoodNanos) * 1e-9, 0.1);
        lastHoodNanos = now;

        double up   = applyDeadband(controller2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), HOOD_TRIGGER_DEADBAND);
        double down = applyDeadband(controller2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),  HOOD_TRIGGER_DEADBAND);

        // Move the target at full speed based on trigger input
        hoodTargetDeg += (up - down) * HOOD_DEG_PER_SEC * dt;
        hoodTargetDeg  = MathUtils.clamp(hoodTargetDeg,
                HoodConstants.Limits.hoodHardStopMin,
                HoodConstants.Limits.hoodHardStopMax);

        // Low-pass filter: smoothly interpolate current toward target each loop.
        // Eliminates jitter from noisy trigger analog values without sacrificing speed.
        hoodCurrentDeg = hoodCurrentDeg + SMOOTH_FACTOR * (hoodTargetDeg - hoodCurrentDeg);

        robot.hood.setAngle(hoodCurrentDeg);
    }

    public double getHoodTargetDeg()  { return hoodTargetDeg;  }
    public double getHoodCurrentDeg() { return hoodCurrentDeg; }
    public double getSelectedRPM()    { return selectedRPM;    }

    private static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) <= deadband) return 0.0;
        return Math.signum(value) * (Math.abs(value) - deadband) / (1.0 - deadband);
    }
}