package org.firstinspires.ftc.teamcode.lib.orchestration;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.constants.HoodConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterFeedConstants;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.lib.triggers.SupplierTrigger;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeed;
import com.seattlesolvers.solverslib.util.MathUtils;

import java.util.function.Predicate;

public class ShotController {
    private SupplierTrigger _shotStagedTrigger;
    private SupplierTrigger _readyToShootTrigger;
    private SupplierTrigger _presetShotModeTrigger;
    private SupplierTrigger _isShootingTrigger;
    private SupplierTrigger _shooterAtRPMTrigger;
    private SupplierTrigger _hoodAtAngleTrigger;
    private SupplierTrigger _robotAtHeadingTrigger;

    private ShotCycleState currentState = ShotCycleState.IDLE;
    private ShotCycleState priorState   = ShotCycleState.IDLE;

    private boolean stateTransition;

    private final NovaPyraRobot robot;
    private final Telemetry     telemetry;

    private double  targetFlywheelRPM = ShooterConstants.Targets.idleRPM;
    private double  targetHoodAngle   = HoodConstants.Limits.hoodHardStopMin;
    private double  targetFeedPower   = ShooterFeedConstants.Targets.defaultFeedPower;
    private boolean presetShotMode    = false;

    private final InterpLUT baselineRPMLUT;
    private final InterpLUT v0CorrectionLUT;
    private final InterpLUT v0SlopeLUT;
    private final InterpLUT v0InterceptLUT;

    public ShotController(NovaPyraRobot robot, Telemetry telemetry) {
        this.robot     = robot;
        this.telemetry = telemetry;

        this.v0CorrectionLUT = setupV0CorrectionLUT();
        this.baselineRPMLUT  = setupBaselineRPMLUT();
        this.v0SlopeLUT      = setupV0SlopeLUT();
        this.v0InterceptLUT  = setupV0InterceptLUT();
    }

    public void setState(ShotCycleState newState) {
        this.currentState = newState;
    }

    public ShotCycleState getState() {
        return this.currentState;
    }

    // --------------------------
    // State machine
    // --------------------------

    public void updateState() {
        if (this.priorState != this.currentState) {
            this.stateTransition = true;
        } else if (this.stateTransition) {
            this.stateTransition = false;
        }

        /*
        telemetry.addData("ShotController: Prior State",       this.priorState.name());
        telemetry.addData("ShotController: Current State",     this.currentState.name());
        telemetry.addData("ShotController: Transitioning?",    this.stateTransition);
        telemetry.addData("ShotController: RPM Target",        this.getTargetFlywheelRPM());
        telemetry.addData("ShotController: Hood Target",       this.getTargetHoodAngle());
        telemetry.addData("ShotController: Feed Power Target", this.getTargetFeedPower());
        telemetry.addData("ShotController: Preset Mode?",      this.presetShotMode);
        */

        this.priorState = this.currentState;

        this.robot.indicatorLights.updatePresetModeWarningLight(this.presetShotMode);

        switch (this.currentState) {
            case OFF:
                break;

            case IDLE:
                if (this.staged()) {
                    this.currentState = ShotCycleState.STAGED;
                }
                break;

            case INTAKE:
                if (robot.sensors.seeingSomeBall(Sensors.SensorLocation.FeedHigh)) {
                    this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
                }
                if (this.robot.intake.isStalled()) {
                    this.robot.intake.setState(Intake.IntakeState.Stopped);
                    this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
                }
                break;

            case STAGED:
                break;

            case PREPARE_SHOT:
                runPrepareShotLogic();
                if (this.readyToShoot()) {
                    this.currentState = ShotCycleState.READY_TO_SHOOT;
                }
                break;

            case READY_TO_SHOOT:
                runPrepareShotLogic();
                if (!this.readyToShoot()) {
                    this.currentState = ShotCycleState.PREPARE_SHOT;
                }
                break;

            case SHOOTING:
                runPrepareShotLogic();
                // Trigger RECOVERY only when the physics solver cannot produce
                // a valid hood angle for the current distance + actual RPM.
                // Normal RPM drop between shots is handled by hood compensation
                // and does NOT trigger recovery.
                if (!canMakeShot()) {
                    this.currentState = ShotCycleState.RECOVERY;
                }
                break;

            case RECOVERY:
                runPrepareShotLogic();
                // Resume SHOOTING once the solver can make the shot again.
                if (canMakeShot()) {
                    this.currentState = ShotCycleState.SHOOTING;
                }
                break;
        }
    }

    /**
     * Shared logic for PREPARE_SHOT, READY_TO_SHOOT, SHOOTING, and RECOVERY.
     * Calls autoTargetShot() every loop unless in preset mode.
     * autoTargetShot() sets targetFlywheelRPM and commands the hood directly.
     */
    private void runPrepareShotLogic() {
        if (!presetShotMode) {
            autoTargetShot();
        }

        robot.shooter.setTargetRPM(this.getTargetFlywheelRPM());
        // Hood is commanded inside autoTargetShot() via setLaunchAngle().
        // In preset mode, command hood via legacy servo angle target.
        if (presetShotMode) {
            robot.hood.setAngle(this.getTargetHoodAngle());
        }
    }

    // --------------------------
    // Physics Solver
    // --------------------------

    /**
     * Main solver entry point — replaces old LUT-based autoTargetShot().
     *
     * Strategy:
     *   - Commands baseline RPM from LUT (always above minimum needed at that distance).
     *   - Reads ACTUAL RPM (may be less than baseline due to drop between shots).
     *   - Solves for the hood launch angle that puts the ball's descending arc
     *     through the goal center at the current distance, given actual v0.
     *   - Commands that angle via robot.hood.setLaunchAngle().
     */
    public void autoTargetShot() {
        this.presetShotMode = false;

        double rawDistance = getDistanceToTarget();

        if (Double.isNaN(rawDistance) || Double.isInfinite(rawDistance)) {
            telemetry.addData("Autoshoot", "No valid distance — skipping solver");
            return;
        }

        double distanceIn = MathUtils.clamp(
                rawDistance / 2.54,
                ShooterConstants.Solver.minDistanceIn,
                ShooterConstants.Solver.maxDistanceIn);

        // Baseline RPM from LUT — tuned to sit above minimum needed at each distance
        double baseline = baselineRPMLUT.get(distanceIn) + ShooterConstants.Solver.baselineOffset;
        setTargetFlywheelRPM(baseline);

        // Solve hood angle from ACTUAL RPM — may have dropped below baseline
        double actualRPM      = robot.shooter.getFlywheelRPM();
        double launchAngleDeg = solveHoodAngle(distanceIn, actualRPM);

        robot.hood.setLaunchAngle(launchAngleDeg);

        telemetry.addData("Autoshoot Distance (in)",     distanceIn);
        telemetry.addData("Autoshoot Baseline RPM",      baseline);
        telemetry.addData("Autoshoot Actual RPM",        actualRPM);
        telemetry.addData("Autoshoot Launch Angle (°)",  launchAngleDeg);
        telemetry.addData("Autoshoot Can Make Shot",     canMakeShot(distanceIn, actualRPM));
    }

    /**
     * Solves for the launch angle (degrees above horizontal) that puts the
     * ball's DESCENDING arc through the goal center at the given distance.
     *
     * Physics:
     *   y = x*tan(α) - [g*x²/(2*v0²*cos²(α))]
     *
     * Rearranged as quadratic in tan(α), high-arc root:
     *   tan(α) = (x + sqrt(x² - 4*Ac*y)) / (2*Ac)
     *   where Ac = g*x²/(2*v0²)
     *
     * Returns clamped launch angle — always safe to command.
     * canMakeShot() catches out-of-range cases and triggers RECOVERY.
     */
    private double solveHoodAngle(double distanceIn, double actualRPM) {
        double v0   = rpmToV0(actualRPM, distanceIn);
        double y    = launchHeightToGoal();

        double Ac   = ShooterConstants.Solver.G * distanceIn * distanceIn
                / (2.0 * v0 * v0);
        double disc = distanceIn * distanceIn - 4.0 * Ac * y;

        if (disc < 0) {
            // v0 too low to reach goal height — return flattest angle.
            // canMakeShot() will catch this and trigger RECOVERY.
            return HoodConstants.Limits.launchAngleMin;  // 48.25°
        }

        double tanAlpha = (distanceIn + Math.sqrt(disc)) / (2.0 * Ac);
        double alphaDeg = Math.toDegrees(Math.atan(tanAlpha));

        return MathUtils.clamp(
                alphaDeg,
                HoodConstants.Limits.launchAngleMin,   // 48.25° — flattest
                HoodConstants.Limits.launchAngleMax);  // 58.0°  — steepest (software clamp)
    }

    /**
     * Returns true if the current distance + actual RPM produce a valid
     * launch angle within the software range [48.25°, 58.0°].
     * Public overload uses live distance + live RPM readings.
     */
    public boolean canMakeShot() {
        double rawDistance = getDistanceToTarget();
        if (Double.isNaN(rawDistance) || Double.isInfinite(rawDistance)) return false;

        double distanceIn = MathUtils.clamp(
                rawDistance / 2.54,
                ShooterConstants.Solver.minDistanceIn,
                ShooterConstants.Solver.maxDistanceIn);

        return canMakeShot(distanceIn, robot.shooter.getFlywheelRPM());
    }

    private boolean canMakeShot(double distanceIn, double actualRPM) {
        double v0   = rpmToV0(actualRPM, distanceIn);
        double y    = launchHeightToGoal();
        double Ac   = ShooterConstants.Solver.G * distanceIn * distanceIn
                / (2.0 * v0 * v0);
        double disc = distanceIn * distanceIn - 4.0 * Ac * y;

        if (disc < 0) return false;

        double tanAlpha = (distanceIn + Math.sqrt(disc)) / (2.0 * Ac);
        double alphaDeg = Math.toDegrees(Math.atan(tanAlpha));

        return alphaDeg >= HoodConstants.Limits.launchAngleMin   // >= 48.25°
                && alphaDeg <= HoodConstants.Limits.launchAngleMax;  // <= 58.0°
    }

    /**
     * Converts flywheel RPM to estimated launch velocity v0 (in/s).
     *
     * v0 = (slope(launchAngle) * RPM + intercept(launchAngle)) * correction(distance)
     *
     * slope/intercept come from per-angle calibration LUTs.
     * correction is a distance-keyed multiplier — tune empirically after goal shots.
     * All correction values start at 1.0 (no correction).
     */
    private double rpmToV0(double rpm, double distanceIn) {
        double launchAngle = robot.hood.getCurrentLaunchAngle();
        double slope       = v0SlopeLUT.get(launchAngle);
        double intercept   = v0InterceptLUT.get(launchAngle);
        double v0          = slope * rpm + intercept;
        double correction  = v0CorrectionLUT.get(distanceIn);
        return v0 * (correction + ShooterConstants.Solver.correction);
    }

    /**
     * Returns y — vertical distance from launch point to goal center (inches).
     * Goal center = 41.25 in (38.75 lip + 2.5 ball radius).
     * Launch height is fixed average (15.39 in) — variation is negligible.
     */
    private double launchHeightToGoal() {
        return ShooterConstants.Solver.goalCenterHeight
                - robot.hood.interpolateLaunchHeight();
    }

    // --------------------------
    // LUT setup
    // --------------------------

    /**
     * v0 correction LUT — distance-keyed scalar multiplier on v0.
     * Corrects for real-world deviation from the empirical calibration curve.
     *
     * Tuning guide:
     *   Shot lands SHORT → increase multiplier at that distance
     *   Shot lands LONG  → decrease multiplier at that distance
     */
    private InterpLUT setupV0CorrectionLUT() {
        InterpLUT lut = new InterpLUT();

        //TODO Tune after goal shots
        lut.add(0,   1.0);
        lut.add(35,  1.0);
        lut.add(80,  1.0);
        lut.add(100, 1.0);
        lut.add(120, 0.8);
        lut.add(140, 0.8);
        lut.add(160, 0.8);
        lut.add(200, 0.8);

        lut.createLUT();
        return lut;
    }

    /**
     * Baseline RPM LUT — distance-keyed.
     * Set above the minimum RPM needed at each distance so the hood has
     * headroom to flatten when RPM drops between shots.
     *
     * Tuning guide: If hood angle is too steep → raise baseline at that distance.
     *               If spin-up takes too long  → lower baseline at that distance.
     */
    private InterpLUT setupBaselineRPMLUT() {
        InterpLUT lut = new InterpLUT();
        lut.add(0,   2400);
        lut.add(35,  2400);  // min ~2385
        lut.add(50,  2280);  // min ~2239
        lut.add(80,  2520);  // min ~2494
        lut.add(100, 2730);  // min ~2708
        lut.add(120, 2900);
        lut.add(140, 3100);
        lut.add(160, 3290);
        lut.add(200, 3290);
        lut.createLUT();
        return lut;
    }

    /**
     * v0 slope LUT — launch angle keyed (degrees).
     * Captures how RPM→v0 sensitivity changes with hood position
     * due to varying ball wrap around the flywheel.
     *
     * Entries cover the full software range (48.25°–58.0°).
     * TODO: Replace placeholder values with real calibration data
     *       from ShooterCalibration.xlsx Curve Fit sheet.
     */
    private InterpLUT setupV0SlopeLUT() {
        InterpLUT lut = new InterpLUT();

        // Bounds — clamp protection outside operating range
        lut.add(0,     0.0934);
        // Operating range 48.25°–58.0°
        // TODO: Fill with real per-angle calibration data
        lut.add(48.25, 0.0735);  // placeholder — use 56° calibration as starting point
        lut.add(50.5,  0.0776);  // placeholder
        lut.add(53.0,  0.0768);  // placeholder
        lut.add(55.5,  0.0724);  // placeholder
        lut.add(58.0,  0.0748);  // placeholder
        // Upper bound clamp
        lut.add(100.0, 0.0768);

        lut.createLUT();
        return lut;
    }

    /**
     * v0 intercept LUT — launch angle keyed (degrees).
     * See setupV0SlopeLUT() for context.
     *
     * TODO: Replace placeholder values with real calibration data
     *       from ShooterCalibration.xlsx Curve Fit sheet.
     */
    private InterpLUT setupV0InterceptLUT() {
        InterpLUT lut = new InterpLUT();

        // Bounds — clamp protection outside operating range
        lut.add(0,     16.7);
        // Operating range 48.25°–58.0°
        // TODO: Fill with real per-angle calibration data
        lut.add(48.25, 26.0);  // placeholder — use 56° calibration as starting point
        lut.add(50.5,  15.5);  // placeholder
        lut.add(53.0,  15.5);  // placeholder
        lut.add(55.5,  26.6);  // placeholder
        lut.add(58.0,  14.6);  // placeholder
        // Upper bound clamp
        lut.add(100.0, 14.6);

        lut.createLUT();
        return lut;
    }

    // --------------------------
    // Distance to target
    // --------------------------

    /**
     * Returns distance to goal in CM.
     * Uses odometry/Pedro as primary source.
     * Limelight direct distance commented out — enable when vision tuned.
     */
    public double getDistanceToTarget() {
        //if (robot.vision.isSeeingTarget()) {
        //    return robot.vision.distanceToTarget();
        //}

        Pose robotPose  = robot.pedroFollower.getPose();
        Pose targetPose = robot.vision.getTargetTagPose();

        if (targetPose == null) {
            return Double.NaN;
        }

        double dxIn = targetPose.getX() - robotPose.getX();
        double dyIn = targetPose.getY() - robotPose.getY();

        return Math.hypot(dxIn, dyIn) * 2.54;  // inches → cm
    }

    // --------------------------
    // Preset shots
    // --------------------------

    public void setupPresetShot(PresetShot shotSetting) {
        this.presetShotMode    = true;
        this.targetFlywheelRPM = shotSetting.targetFlywheelRPM;
        this.targetHoodAngle   = shotSetting.targetHoodAngle;
        this.targetFeedPower   = shotSetting.targetFeedPower;
    }

    // --------------------------
    // Targets getters/setters
    // --------------------------

    public void setTargetFlywheelRPM(double newTarget) { this.targetFlywheelRPM = newTarget; }
    public void setTargetHoodAngle(double newTarget)   { this.targetHoodAngle   = newTarget; }
    public void setTargetFeedPower(double newTarget)   { this.targetFeedPower   = newTarget; }

    public double getTargetFlywheelRPM() { return this.targetFlywheelRPM; }
    public double getTargetHoodAngle()   { return this.targetHoodAngle;   }
    public double getTargetFeedPower()   { return this.targetFeedPower;   }

    // --------------------------
    // Conditions and helpers
    // --------------------------

    public boolean staged() {
        return this.robot.sensors.seeingSomeBall();
    }

    private boolean atRPM() {
        return this.robot.shooter.atRPM();
    }

    private boolean atTargetAngle() {
        return this.robot.hood.atTargetAngle();
    }

    private boolean atTargetHeading() {
        Pose currentPose = robot.pedroFollower.getPose();
        Pose targetPose  = robot.vision.getTargetTagPose();

        double dx            = targetPose.getX() - currentPose.getX();
        double dy            = targetPose.getY() - currentPose.getY();
        double targetHeading = Math.atan2(dy, dx);

        double headingError    = normalizeAngle(targetHeading - currentPose.getHeading());
        double headingErrorDeg = Math.toDegrees(Math.abs(headingError));

        return headingErrorDeg <= VisionConstants.headingAngleThresholdDeg;
    }

    private double normalizeAngle(double angleRad) {
        while (angleRad > Math.PI)   angleRad -= 2.0 * Math.PI;
        while (angleRad <= -Math.PI) angleRad += 2.0 * Math.PI;
        return angleRad;
    }

    public boolean readyToShoot() {
        return this.atRPM() && this.atTargetAngle() && this.atTargetHeading();
    }

    // Retained for any external callers — delegates to canMakeShot()
    public boolean isRPMatTarget() {
        return canMakeShot();
    }

    public boolean transitionedTo(ShotCycleState newState) {
        return this.stateTransition && this.currentState == newState;
    }

    public boolean doAutoHeading() {
        return this.currentState == ShotCycleState.PREPARE_SHOT   ||
                this.currentState == ShotCycleState.READY_TO_SHOOT ||
                this.currentState == ShotCycleState.SHOOTING       ||
                this.currentState == ShotCycleState.RECOVERY;
    }

    public boolean isShooting() {
        return this.currentState == ShotCycleState.PREPARE_SHOT   ||
                this.currentState == ShotCycleState.READY_TO_SHOOT ||
                this.currentState == ShotCycleState.SHOOTING       ||
                this.currentState == ShotCycleState.RECOVERY;
    }

    // --------------------------
    // Triggers
    // --------------------------

    public Trigger shotStagedTrigger() {
        if (this._shotStagedTrigger == null)
            this._shotStagedTrigger = new SupplierTrigger(this::staged);
        return this._shotStagedTrigger;
    }

    public Trigger readyToShootTrigger() {
        if (this._readyToShootTrigger == null)
            this._readyToShootTrigger = new SupplierTrigger(this::readyToShoot);
        return this._readyToShootTrigger;
    }

    public Trigger isShootingTrigger() {
        if (this._isShootingTrigger == null)
            this._isShootingTrigger = new SupplierTrigger(this::isShooting);
        return this._isShootingTrigger;
    }

    public Trigger shooterAtRPM() {
        if (this._shooterAtRPMTrigger == null)
            this._shooterAtRPMTrigger = new SupplierTrigger(this::atRPM);
        return this._shooterAtRPMTrigger;
    }

    public Trigger hoodAtAngle() {
        if (this._hoodAtAngleTrigger == null)
            this._hoodAtAngleTrigger = new SupplierTrigger(this::atTargetAngle);
        return this._hoodAtAngleTrigger;
    }

    public Trigger robotAtHeading() {
        if (this._robotAtHeadingTrigger == null)
            this._robotAtHeadingTrigger = new SupplierTrigger(this::atTargetHeading);
        return this._robotAtHeadingTrigger;
    }

    public StateTransitionToTrigger stateTransitionTrigger(ShotCycleState toState) {
        return new StateTransitionToTrigger(this::transitionedTo, toState);
    }

    public static class StateTransitionToTrigger extends Trigger {
        ShotCycleState            checkState;
        Predicate<ShotCycleState> getFn;

        public StateTransitionToTrigger(Predicate<ShotCycleState> shotCycleStateSupplier,
                                        ShotCycleState state) {
            this.checkState = state;
            this.getFn      = shotCycleStateSupplier;
        }

        @Override
        public boolean get() { return this.getFn.test(checkState); }
    }

    // --------------------------
    // State transition side-effects
    // --------------------------

    public void setupStateTransitionTriggers() {

        stateTransitionTrigger(ShotCycleState.OFF).whenActive(new InstantCommand(() -> {
            this.robot.intake.setState(Intake.IntakeState.Stopped);
            this.robot.feedStop.engage();
            this.robot.shooter.setState(Shooter.ShooterState.Stopped);
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
            this.robot.hood.setAngle(HoodConstants.Limits.hoodHardStopMin);
        }));

        stateTransitionTrigger(ShotCycleState.IDLE).whenActive(new InstantCommand(() -> {
            this.robot.intake.setState(Intake.IntakeState.Stopped);
            this.robot.feedStop.engage();
            this.robot.shooter.setState(Shooter.ShooterState.Shooting);
            this.robot.shooter.setTargetRPM(ShooterConstants.Targets.idleRPM);
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
            this.robot.hood.setAngle(HoodConstants.Limits.hoodHardStopMin);
        }));

        stateTransitionTrigger(ShotCycleState.INTAKE).whenActive(new InstantCommand(() -> {
            this.robot.intake.setState(Intake.IntakeState.Intake);
            this.robot.feedStop.engage();
            this.robot.shooter.setState(Shooter.ShooterState.Shooting);
            this.robot.shooter.setTargetRPM(ShooterConstants.Targets.idleRPM);
            this.robot.shooterFeed.setFeedPower(this.getTargetFeedPower());
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Feeding);
            this.robot.hood.setAngle(HoodConstants.Limits.hoodHardStopMin);
        }));

        stateTransitionTrigger(ShotCycleState.STAGED).whenActive(new InstantCommand(() -> {
            this.robot.intake.setState(Intake.IntakeState.Stopped);
            this.robot.feedStop.engage();
            this.robot.shooter.setState(Shooter.ShooterState.Shooting);
            this.robot.shooter.setTargetRPM(ShooterConstants.Targets.idleRPM);
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
            this.robot.hood.setAngle(HoodConstants.Limits.hoodHardStopMin);
        }));

        stateTransitionTrigger(ShotCycleState.PREPARE_SHOT).whenActive(new InstantCommand(() -> {
            this.robot.feedStop.engage();
            this.robot.shooter.setState(Shooter.ShooterState.Shooting);
            this.robot.intake.setState(Intake.IntakeState.Stopped);
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
            // Hood commanded each loop by runPrepareShotLogic() → autoTargetShot()
        }));

        stateTransitionTrigger(ShotCycleState.READY_TO_SHOOT).whenActive(new InstantCommand(() -> {
            this.robot.feedStop.engage();
            this.robot.shooter.setState(Shooter.ShooterState.Shooting);
            this.robot.intake.setState(Intake.IntakeState.Stopped);
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
            // Hood commanded each loop by runPrepareShotLogic() → autoTargetShot()
        }));

        stateTransitionTrigger(ShotCycleState.SHOOTING).whenActive(new InstantCommand(() -> {
            this.robot.feedStop.disengage();
            this.robot.shooter.setState(Shooter.ShooterState.Shooting);
            this.robot.shooterFeed.setFeedPower(this.getTargetFeedPower());
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Feeding);
            this.robot.intake.setState(Intake.IntakeState.Intake);
            // Hood commanded each loop by runPrepareShotLogic() → autoTargetShot()
        }));

        stateTransitionTrigger(ShotCycleState.RECOVERY).whenActive(new InstantCommand(() -> {
            // Keep gate open — re-engaging risks jamming
            // Stop feed while RPM recovers
            this.robot.feedStop.disengage();
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
            this.robot.intake.setState(Intake.IntakeState.Stopped);
            this.robot.shooter.setState(Shooter.ShooterState.Shooting);
            // Hood commanded each loop by runPrepareShotLogic() → autoTargetShot()
        }));
    }
}