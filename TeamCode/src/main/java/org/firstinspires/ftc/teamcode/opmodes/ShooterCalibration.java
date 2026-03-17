package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.constants.Alliance;
import org.firstinspires.ftc.teamcode.constants.HoodConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterFeedConstants;
import org.firstinspires.ftc.teamcode.lib.triggers.TriggerAxis;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeed;

/**
 * ShooterCalibration OpMode
 *
 * Purpose: Empirically characterize the relationship between
 *   flywheel RPM + hood angle → actual ball launch velocity (v0).
 *
 * Method:
 *   1. Set a target RPM and hood angle using controller 2.
 *   2. Hold C1 Left Bumper to spin up. Wait for "✓ AT SPEED".
 *   3. Hold C1 Right Trigger to feed. Release when RPM dip recovers
 *      (RPM dip = ball contacting flywheel; recovery = ball has exited).
 *   4. Measure horizontal range to where ball landed.
 *   5. Press C2 B to log the shot in telemetry.
 *   6. Record data in spreadsheet and back-calculate v0.
 *
 * Back-calculate v0 formula (use in Excel/Sheets after data collection):
 *   v0 = sqrt( g * x^2 / (2 * cos^2(alpha) * (x * tan(alpha) - y)) )
 *   where:
 *     g     = 386.1 in/s^2
 *     x     = measured horizontal range (inches)
 *     y     = -18.0 inches (floor is 18" below launch point)
 *     alpha = hood angle in RADIANS
 *
 * Controller Layout:
 *   Controller 1 (Driver):
 *     Left Bumper  (hold) : Spin up flywheel to target RPM
 *     Right Trigger (hold): Run feed — release after RPM dip recovers
 *     Start               : Emergency stop everything
 *
 *   Controller 2 (Operator):
 *     Right Stick Y       : RPM ±50 (continuous, per loop)
 *     D-Pad Up            : RPM +200
 *     D-Pad Down          : RPM -200
 *     Left Stick Y        : Hood ±1 degree (continuous, per loop)
 *     D-Pad Right         : Hood +5 degrees
 *     D-Pad Left          : Hood -5 degrees
 *     Y Button            : Snapshot current RPM + hood to log
 *     B Button            : Log last shot (RPM at fire + hood angle)
 *
 * Suggested calibration matrix:
 *   Hood angles : 15, 20, 25, 30, 34 degrees
 *   RPM per angle: low / mid / high end of operating range
 *   3 shots per combination — average the measured range
 */
@TeleOp(name = "Shooter Calibration", group = "Calibration")
public class ShooterCalibration extends CommandOpMode {

    // -------------------------------------------------------------------------
    // Constants
    // -------------------------------------------------------------------------

    private static final double HOOD_MIN_DEG        = HoodConstants.Limits.hoodHardStopMin;
    private static final double HOOD_MAX_DEG        = HoodConstants.Limits.hoodHardStopMax;
    private static final double RPM_MIN             = 1500.0;
    private static final double RPM_MAX             = 3500.0;
    private static final double RPM_FINE            = 50.0;
    private static final double RPM_COARSE          = 100.0;
    private static final double HOOD_FINE           = 1.0;
    private static final double HOOD_COARSE         = 5.0;
    private static final double STICK_DEADBAND      = 0.4;
    private static final double READY_RPM_TOLERANCE = ShooterConstants.flywheelRPMTolerance;
    private static final double FEED_POWER          = ShooterFeedConstants.Targets.defaultFeedPower;

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------

    private NovaPyraRobot robot;
    private GamepadEx     controller1;
    private GamepadEx     controller2;

    private double  targetRPM      = 2000.0;
    private double  targetHoodDeg  = 25.0;
    private boolean spinUpEnabled  = false;
    private double  rpmAtLastFire  = 0.0;
    private double  hoodAtLastFire = 0.0;
    private int     shotCount      = 0;
    private String  lastLogLine    = "None";

    // -------------------------------------------------------------------------
    // Initialize — subsystems + all button/trigger bindings
    // -------------------------------------------------------------------------

    @Override
    public void initialize() {
        super.reset();

        robot       = new NovaPyraRobot(hardwareMap, telemetry, Alliance.BLUE);
        controller1 = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);

        // Safe starting state
        robot.shooter.setState(Shooter.ShooterState.Stopped);
        robot.kicker.setState(Kicker.KickerState.Stopped);
        robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
        robot.hood.setAngle(targetHoodDeg);

        setupButtonBindings();

        telemetry.addLine("=== SHOOTER CALIBRATION READY ===");
        telemetry.addLine("Hold C1 LB to spin up. Hold C1 RT to feed.");
        telemetry.update();
    }

    private void setupButtonBindings() {

        // ── Controller 1 ─────────────────────────────────────────────────────

        GamepadButton c1LeftBumper   = new GamepadButton(controller1, GamepadKeys.Button.LEFT_BUMPER);
        GamepadButton c1Start        = new GamepadButton(controller1, GamepadKeys.Button.START);
        TriggerAxis   c1RightTrigger = new TriggerAxis(controller1,
                GamepadKeys.Trigger.RIGHT_TRIGGER, 0.5);

        // Left Bumper: hold to spin up, release to stop
        c1LeftBumper.whenActive(new InstantCommand(() -> {
            spinUpEnabled = true;
            robot.shooter.setState(Shooter.ShooterState.Shooting);
            robot.kicker.setState(Kicker.KickerState.Spinning);
        }));
        c1LeftBumper.whenInactive(new InstantCommand(() -> {
            spinUpEnabled = false;
            robot.shooter.setState(Shooter.ShooterState.Stopped);
            robot.kicker.setState(Kicker.KickerState.Stopped);
            robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
        }));

        // Right Trigger: hold to feed, release to stop
        // RPM + hood are snapshotted on the active edge
        c1RightTrigger.whenActive(new InstantCommand(() -> {
            if (spinUpEnabled && isAtTargetRPM()) {
                rpmAtLastFire  = robot.shooter.getFlywheelRPM();
                hoodAtLastFire = targetHoodDeg;
                shotCount++;
                robot.shooterFeed.setFeedPower(FEED_POWER);
                robot.shooterFeed.setState(ShooterFeed.FeedState.Feeding);
                robot.intake.setState(Intake.IntakeState.Intake);
            }
        }));
        c1RightTrigger.whenInactive(new InstantCommand(() -> {
            robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
            robot.intake.setState(Intake.IntakeState.Stopped);
        }));

        // Start: emergency stop
        c1Start.whenActive(new InstantCommand(this::stopAll));

        GamepadButton c1DPadUp    = new GamepadButton(controller1, GamepadKeys.Button.DPAD_UP);
        GamepadButton c1DPadDown  = new GamepadButton(controller1, GamepadKeys.Button.DPAD_DOWN);
        GamepadButton c1DPadLeft  = new GamepadButton(controller1, GamepadKeys.Button.DPAD_LEFT);
        GamepadButton c1DPadRight = new GamepadButton(controller1, GamepadKeys.Button.DPAD_RIGHT);
        GamepadButton c1Y         = new GamepadButton(controller1, GamepadKeys.Button.Y);
        GamepadButton c1B         = new GamepadButton(controller1, GamepadKeys.Button.B);

        // Coarse RPM adjustment
        c1DPadUp.whenActive(new InstantCommand(() ->
                targetRPM = MathUtils.clamp(targetRPM + RPM_COARSE, RPM_MIN, RPM_MAX)));
        c1DPadDown.whenActive(new InstantCommand(() ->
                targetRPM = MathUtils.clamp(targetRPM - RPM_COARSE, RPM_MIN, RPM_MAX)));

        // Coarse hood adjustment
        c1DPadRight.whenActive(new InstantCommand(() ->
                targetHoodDeg = MathUtils.clamp(targetHoodDeg + HOOD_COARSE, HOOD_MIN_DEG, HOOD_MAX_DEG)));
        c1DPadLeft.whenActive(new InstantCommand(() ->
                targetHoodDeg = MathUtils.clamp(targetHoodDeg - HOOD_COARSE, HOOD_MIN_DEG, HOOD_MAX_DEG)));

        // Y: snapshot current settings to log
        c1Y.whenActive(new InstantCommand(() ->
                lastLogLine = String.format(
                        "SNAP | RPM tgt=%.0f act=%.0f | Hood=%.1f deg",
                        targetRPM, robot.shooter.getFlywheelRPM(), targetHoodDeg)
        ));

        // B: log last shot
        c1B.whenActive(new InstantCommand(() ->
                lastLogLine = String.format(
                        "SHOT #%d | RPM at fire=%.0f | Hood=%.1f deg | Measure range now!",
                        shotCount, rpmAtLastFire, hoodAtLastFire)
        ));
    }

    // -------------------------------------------------------------------------
    // Run loop — analog stick adjustments + apply targets
    // -------------------------------------------------------------------------

    @Override
    public void run() {
        super.run();

        // Fine RPM adjustment — right stick Y
        double c1RightY = controller1.getRightY();
        if (Math.abs(c1RightY) > STICK_DEADBAND) {
            targetRPM = MathUtils.clamp(
                    targetRPM + Math.signum(c1RightY) * RPM_FINE,
                    RPM_MIN, RPM_MAX);
        }

        // Fine hood adjustment — left stick Y
        double c1LeftY = controller1.getLeftY();
        if (Math.abs(c1LeftY) > STICK_DEADBAND) {
            targetHoodDeg = MathUtils.clamp(
                    targetHoodDeg + Math.signum(c1LeftY) * HOOD_FINE,
                    HOOD_MIN_DEG, HOOD_MAX_DEG);
        }

        // Apply targets every loop
        if (spinUpEnabled) {
            robot.shooter.setTargetRPM(targetRPM);
            robot.kicker.setTargetRPMFromFlywheel(targetRPM);
        }
        robot.hood.setAngle(targetHoodDeg);

        // ── Telemetry ─────────────────────────────────────────────────────────

        boolean atRPM       = isAtTargetRPM();
        boolean readyToFire = spinUpEnabled && atRPM;

        telemetry.addLine("════ SHOOTER CALIBRATION ════");
        telemetry.addLine("");
        telemetry.addData("Spin Up",        spinUpEnabled ? "ENABLED (LB held)" : "OFF");
        telemetry.addData("READY TO FIRE",  readyToFire   ? "✓ YES — hold RT"   : "✗ NO");
        telemetry.addLine("");
        telemetry.addData("RPM  Target",    String.format("%.0f", targetRPM));
        telemetry.addData("RPM  Actual",    String.format("%.0f", robot.shooter.getFlywheelRPM()));
        telemetry.addData("RPM  Kicker",    String.format("%.0f", robot.kicker.getKickerRPM()));
        telemetry.addData("RPM  Status",    atRPM ? "✓ AT SPEED" : "... spinning up");
        telemetry.addLine("");
        telemetry.addData("Hood Target",    String.format("%.1f deg", targetHoodDeg));
        telemetry.addData("Launch Angle", String.format("%.1f deg", robot.hood.getCurrentLaunchAngle()));
        telemetry.addLine("");
        telemetry.addData("Shot Count",     shotCount);
        telemetry.addData("Last Fire RPM",  String.format("%.0f", rpmAtLastFire));
        telemetry.addData("Last Fire Hood", String.format("%.1f deg", hoodAtLastFire));
        telemetry.addLine("");
        telemetry.addLine("── Log ──");
        telemetry.addData("", lastLogLine);
        telemetry.addLine("");
        telemetry.addLine("── Controls ──");
        telemetry.addLine("C1: LB=SpinUp  RT=Feed(hold)  Start=STOP");
        telemetry.addLine("C2: RStick=RPM±50  DPadU/D=RPM±200");
        telemetry.addLine("C2: LStick=Hood±1°  DPadL/R=Hood±5°");
        telemetry.addLine("C2: Y=Snapshot  B=LogShot");

        telemetry.update();
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    private boolean isAtTargetRPM() {
        return Math.abs(robot.shooter.getFlywheelRPM() - targetRPM) <= READY_RPM_TOLERANCE;
    }

    private void stopAll() {
        spinUpEnabled = false;
        robot.shooter.setState(Shooter.ShooterState.Stopped);
        robot.kicker.setState(Kicker.KickerState.Stopped);
        robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
    }

    @Override
    public void reset() {
        robot.reset();
    }
}