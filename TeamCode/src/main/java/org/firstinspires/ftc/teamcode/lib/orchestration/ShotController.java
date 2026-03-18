package org.firstinspires.ftc.teamcode.lib.orchestration;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HoodConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterFeedConstants;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.lib.triggers.SupplierTrigger;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
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
    private ShotCycleState priorState = ShotCycleState.IDLE;

    private boolean stateTransition;

    private final NovaPyraRobot robot;
    private final Telemetry telemetry;

    private double targetFlywheelRPM = ShooterConstants.Targets.idleRPM;
    private double targetHoodAngle = HoodConstants.Limits.hoodHardStopMax;
    private double targetFeedPower = ShooterFeedConstants.Targets.defaultFeedPower;
    private boolean presetShotMode = false;

    private final InterpLUT baselineRPMLUT;

    // v0 correction LUT — keyed on distance (inches), value is a scalar multiplier.
    // All values start at 1.0 (no correction). Tune empirically after calibration
    // if shots deviate from predicted range at specific distances.
    // If close shots land long  → decrease the entry for that distance below 1.0
    // If far shots land short   → increase the entry for that distance above 1.0
    private final InterpLUT v0CorrectionLUT;

    private final InterpLUT v0SlopeLUT;
    private final InterpLUT v0InterceptLUT;

    public ShotController(NovaPyraRobot robot, Telemetry telemetry) {
        this.robot    = robot;
        this.telemetry = telemetry;

        this.v0CorrectionLUT = setupV0CorrectionLUT();
        this.baselineRPMLUT  = setupBaselineRPMLUT();
        this.v0SlopeLUT = setupv0SlopeLut();
        this.v0InterceptLUT = setupv0InterceptLUT();
    }

    public void setState(ShotCycleState newState) {
        this.currentState = newState;
    }

    public ShotCycleState getState() {
        return this.currentState;
    }

    public void updateState() {
        // Flag state transitions
        if (this.priorState != this.currentState) {
            this.stateTransition = true;
        } else if (this.stateTransition) {
            this.stateTransition = false;
        }


        /*
        telemetry.addData("ShotController: Prior State", this.priorState.name());
        telemetry.addData("ShotController: Current State", this.currentState.name());
        telemetry.addData("ShotController: Transitioning?", this.stateTransition);
        telemetry.addData("ShotController: RPM Target", this.getTargetFlywheelRPM());
        telemetry.addData("ShotController: Hood Target", this.getTargetHoodAngle());
        telemetry.addData("ShotController: Feed Power Target", this.getTargetFeedPower());
        telemetry.addData("ShotController: Preset Mode?", this.presetShotMode);

         */

        this.priorState = this.currentState;

        this.robot.indicatorLights.updatePresetModeWarningLight(this.presetShotMode);

        // State actions that need to happen each cycle
        switch (this.currentState) {
            case OFF:
                break;

            case IDLE:
                // From idle, if we detect a staged ball, move to STAGED automatically.
                if (this.staged()) {
                    this.currentState = ShotCycleState.STAGED;
                }
                break;

            case INTAKE:
                // Behavior handled mostly by state transition triggers + LT mapping.
                // Stop feed wheel when feedhigh sensor detects artifact
                if (robot.sensors.seeingSomeBall(Sensors.SensorLocation.FeedHigh)){
                    this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
                }
                // Check if intake is stalled and stop it
                if (this.robot.intake.isStalled()) {
                    this.robot.intake.setState(Intake.IntakeState.Stopped);
                    this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
                }
                break;

            case STAGED:
                // Waiting for driver to start PREPARE_SHOT (via LB).
                break;

            case PREPARE_SHOT:
                runPrepareShotLogic();
                if (this.readyToShoot()) {
                    this.currentState = ShotCycleState.READY_TO_SHOOT;
                }
                break;

            case READY_TO_SHOOT:
                // Keep everything updated; if we drift off, fall back to PREPARE_SHOT
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
                // Resume SHOOTING once the solver can make the shot again
                // (RPM has recovered enough for the hood to compensate).
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
        robot.kicker.setTargetRPMFromFlywheel(this.getTargetFlywheelRPM());
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
     * Physics Solver - replaces the old LUT-based autoTargetShot().
     *
     * Strategy:
     *   - Commands a fixed baseline RPM per zone (always higher than minimum needed).
     *   - Reads ACTUAL RPM (which may be less than baseline due to drop between shots).
     *   - Solves for the hood launch angle that puts the ball's descending arc
     *     through the goal center at the current distance, given actual v0.
     *   - Commands that angle via robot.hood.setLaunchAngle().
     *
     * This gives the hood automatic per-shot compensation for RPM drop —
     * the baseline RPM provides headroom and the hood absorbs the variation.
     */
    public void autoTargetShot() {
        this.presetShotMode = false;

        double rawDistance = getDistanceToTarget();

        // If we don't have a valid distance (no tag, NaN, etc.), don't touch current targets
        if (Double.isNaN(rawDistance) || Double.isInfinite(rawDistance)) {
            telemetry.addData("Autoshoot", "No valid distance — skipping solver");
            return;
        }

        double distanceIn = MathUtils.clamp(
                rawDistance / 2.54,                          // cm → inches
                ShooterConstants.Solver.minDistanceIn,
                ShooterConstants.Solver.maxDistanceIn);


        // Select baseline RPM for this distance from LUT
        double baseline = baselineRPMLUT.get(distanceIn) + ShooterConstants.Solver.baselineOffset;
        setTargetFlywheelRPM(baseline);

        // Solve hood angle from ACTUAL RPM (not target — may have dropped)
        double actualRPM      = robot.shooter.getFlywheelRPM();
        double launchAngleDeg = solveHoodAngle(distanceIn, actualRPM);

        robot.hood.setLaunchAngle(launchAngleDeg);

        telemetry.addData("Autoshoot Distance (in)",    distanceIn);
        telemetry.addData("Autoshoot Zone Baseline RPM", baseline);
        telemetry.addData("Autoshoot Actual RPM",        actualRPM);
        telemetry.addData("Autoshoot Launch Angle (°)",  launchAngleDeg);
        telemetry.addData("Autoshoot Mech Angle (°)",    robot.hood.getCurrentMechanicalAngle());
        telemetry.addData("Autoshoot Can Make Shot",     canMakeShot(distanceIn, actualRPM));
    }

    /**
     * Solves for the launch angle (degrees above horizontal) that puts the
     * ball's DESCENDING arc through the goal center at the given distance.
     *
     * Physics:
     *   y = x*tan(α) - [g*x²/(2*v0²*cos²(α))]
     *
     * Rearranged as quadratic in tan(α):
     *   Ac*tan²(α) - x*tan(α) + y = 0
     *   where Ac = g*x²/(2*v0²)
     *
     * High-arc (descending) root:
     *   tan(α) = (x + sqrt(x² - 4*Ac*y)) / (2*Ac)
     *
     * Returns a clamped launch angle — always safe to command even if
     * the ideal solution is outside the mechanical range.
     * canMakeShot() will catch out-of-range cases and trigger RECOVERY.
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
            return HoodConstants.Limits.launchAngleAtMechMax;  // 56°
        }

        double tanAlpha = (distanceIn + Math.sqrt(disc)) / (2.0 * Ac);
        double alphaDeg = Math.toDegrees(Math.atan(tanAlpha));

        return MathUtils.clamp(
                alphaDeg,
                HoodConstants.Limits.launchAngleAtMechMax,   // 56° — flattest
                HoodConstants.Limits.launchAngleAtMechMin);  // 72.8° — steepest
    }

    /**
     * Returns true if the current distance + actual RPM produce a valid
     * launch angle within the mechanical range [56°, 72.8°].
     *
     * Used by the state machine to decide whether to trigger RECOVERY.
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

        return alphaDeg >= HoodConstants.Limits.launchAngleAtMechMax   // >= 56°
                && alphaDeg <= HoodConstants.Limits.launchAngleAtMechMin;  // <= 72.8°
    }

    /**
     * Converts flywheel RPM to estimated launch velocity v0 (in/s).
     *
     * v0 = (slope * RPM + intercept) * correctionMultiplier(distance)
     *
     * The correction LUT absorbs systematic deviation of the empirical curve
     * from real-world performance across the distance range.
     * All LUT values start at 1.0 — tune empirically after calibration.
     */

    private double rpmToV0(double rpm, double distanceIn) {
        double launchAngle = robot.hood.getCurrentLaunchAngle();
        double slope       = v0SlopeLUT.get(launchAngle) + ShooterConstants.Solver.slope;
        double intercept   = v0InterceptLUT.get(launchAngle) + ShooterConstants.Solver.intercept;
        double v0          = slope * rpm + intercept;
        double correction = v0CorrectionLUT.get(distanceIn) + ShooterConstants.Solver.correction;
        return v0 * correction;
    }

    /**
     * Returns y — vertical distance from launch point to goal center (inches).
     * Goal center height = 41.25 in (38.75 lip + 2.5 ball radius).
     * Launch height interpolated from current mechanical hood angle.
     */
    private double launchHeightToGoal() {
        return ShooterConstants.Solver.goalCenterHeight
                - robot.hood.interpolateLaunchHeight();
    }

    // --------------------------
    // v0 Correction LUT
    // --------------------------

    /**
     * Distance-keyed multiplier on v0 to correct for systematic deviation
     * of the empirical curve from real goal shots.
     *
     * All values start at 1.0 (no correction).
     * Add more distance points if deviation is non-uniform across the range.
     *
     * Tuning guide:
     *   Shot lands SHORT of target → increase multiplier at that distance
     *   Shot lands LONG  of target → decrease multiplier at that distance
     */
    private InterpLUT setupV0CorrectionLUT() {
        InterpLUT lut = new InterpLUT();

        //TODO Tune after calibration
        lut.add(0, 0.88);
        lut.add(35,  0.88);
        lut.add(80,  0.88);
        lut.add(100, 0.88);
        lut.add(120, 0.9);
        lut.add(140, 0.9);
        lut.add(160, 0.9);
        lut.add(200, 0.9);

        lut.createLUT();
        return lut;
    }

    private InterpLUT setupBaselineRPMLUT() {
        InterpLUT lut = new InterpLUT();

        //TODO Tune after calibration — set each point ~200 RPM above the minimum
        // RPM needed at that distance so the hood has downward headroom.
        // Solver Preview sheet will tell you the minimum needed per distance.
        lut.add(0,   1600);
        lut.add(35,  1600);
        lut.add(80,  2020);
        lut.add(100,2195);
        lut.add(120, 2400);
        lut.add(140, 2500);
        lut.add(160, 2750);
        lut.add(200, 2750);

        lut.createLUT();
        return lut;
    }

    private InterpLUT setupv0SlopeLut() {
        InterpLUT lut = new InterpLUT();

        // v0SlopeLUT
        lut.add(0, 0.0934);
        lut.add(56.0,  0.0934);
        lut.add(59.7,  0.0726);
        lut.add(64.0,  0.0787);
        lut.add(68.4,  0.0888);
        lut.add(72.8,  0.0686);
        lut.add(100.0,0.0686);

        lut.createLUT();
        return lut;
    }

    private InterpLUT setupv0InterceptLUT() {
        InterpLUT lut = new InterpLUT();

        // v0InterceptLUT
        lut.add(0, 16.7);
        lut.add(56.0,  16.7);
        lut.add(59.7,  63.0);
        lut.add(64.0,  48.4);
        lut.add(68.4,  16.3);
        lut.add(72.8,  56.8);
        lut.add(100.0,56.8);

        lut.createLUT();
        return lut;
    }


    // --------------------------
    // Distance to target
    // --------------------------

    /**
     * Hybrid distance:
     * - If Limelight sees tag, use LL distance (Vision.distanceToTarget()).
     * - Otherwise, use odometry distance to a known goal point on the field.
     * Returns distance in CM.
     */
    public double getDistanceToTarget() {
        Pose robotPose  = robot.pedroFollower.getPose();
        Pose targetPose = robot.vision.getTargetTagPose();

        if (targetPose == null) {
            return Double.NaN;
        }

        double dxIn = targetPose.getX() - robotPose.getX();
        double dyIn = targetPose.getY() - robotPose.getY();

        double distanceIn = Math.hypot(dxIn, dyIn);
        return distanceIn * 2.54; // inches -> cm
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

    public void setTargetFlywheelRPM(double newTarget) {
        this.targetFlywheelRPM = newTarget;
    }

    public void setTargetHoodAngle(double newTarget) {
        this.targetHoodAngle = newTarget;
    }

    public void setTargetFeedPower(double newTarget) {
        this.targetFeedPower = newTarget;
    }

    public double getTargetFlywheelRPM() {
        return this.targetFlywheelRPM;
    }

    public double getTargetHoodAngle() {
        return this.targetHoodAngle;
    }

    public double getTargetFeedPower() {
        return this.targetFeedPower;
    }

    // --------------------------
    // Conditions and helpers
    // --------------------------

    public boolean staged() {
        return this.robot.sensors.seeingSomeBall();
    }

    private boolean atRPM() {
        return this.robot.shooter.atRPM() && this.robot.kicker.atRPM();
    }

    private boolean atTargetAngle() {
        return this.robot.hood.atTargetAngle();
    }

    private boolean atTargetHeading() {
        // Use odometry-based alignment check from Exploration
        // This matches the auto-aim control logic
        Pose currentPose = robot.pedroFollower.getPose();
        Pose targetPose = robot.vision.getTargetTagPose();

        // Calculate vector from robot to target
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();
        double targetHeading = Math.atan2(dy, dx);

        // Calculate heading error
        double headingError = normalizeAngle(targetHeading - currentPose.getHeading());
        double headingErrorDeg = Math.toDegrees(Math.abs(headingError));

        // Check if within tolerance
        return headingErrorDeg <= VisionConstants.headingAngleThresholdDeg;
    }

    // Add helper method
    private double normalizeAngle(double angleRad) {
        while (angleRad > Math.PI) angleRad -= 2.0 * Math.PI;
        while (angleRad <= -Math.PI) angleRad += 2.0 * Math.PI;
        return angleRad;
    }

    public boolean readyToShoot() {
        return this.atRPM() && this.atTargetAngle() && this.atTargetHeading();
    }

    // isRPMatTarget() retained for any external callers — delegates to canMakeShot()
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
        if (this._shotStagedTrigger == null) {
            this._shotStagedTrigger = new SupplierTrigger(this::staged);
        }
        return this._shotStagedTrigger;
    }

    public Trigger readyToShootTrigger() {
        if (this._readyToShootTrigger == null) {
            this._readyToShootTrigger = new SupplierTrigger(this::readyToShoot);
        }
        return this._readyToShootTrigger;
    }

    public Trigger isShootingTrigger() {
        if (this._isShootingTrigger == null) {
            this._isShootingTrigger = new SupplierTrigger(this::isShooting);
        }
        return this._isShootingTrigger;
    }

    public Trigger shooterAtRPM() {
        if (this._shooterAtRPMTrigger == null) {
            this._shooterAtRPMTrigger = new SupplierTrigger(this::atRPM);
        }
        return this._shooterAtRPMTrigger;
    }

    public Trigger hoodAtAngle() {
        if (this._hoodAtAngleTrigger == null) {
            this._hoodAtAngleTrigger = new SupplierTrigger(this::atTargetAngle);
        }
        return this._hoodAtAngleTrigger;
    }

    public Trigger robotAtHeading() {
        if (this._robotAtHeadingTrigger == null) {
            this._robotAtHeadingTrigger = new SupplierTrigger(this::atTargetHeading);
        }
        return this._robotAtHeadingTrigger;
    }

    public StateTransitionToTrigger stateTransitionTrigger(ShotCycleState toState) {
        return new StateTransitionToTrigger(this::transitionedTo, toState);
    }

    public static class StateTransitionToTrigger extends Trigger {
        ShotCycleState             checkState;
        Predicate<ShotCycleState>  getFn;

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
            // Transitioning back to OFF state
            this.robot.intake.setState(Intake.IntakeState.Stopped);
            this.robot.feedStop.engage();
            this.robot.shooter.setState(Shooter.ShooterState.Stopped);
            this.robot.kicker.setState(Kicker.KickerState.Stopped);
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
            this.robot.hood.setAngle(HoodConstants.Limits.hoodHardStopMin);
        }));

        stateTransitionTrigger(ShotCycleState.IDLE).whenActive(new InstantCommand(() -> {
            // Transitioning back to IDLE state
            // NOTE: This will not trigger at start because initial state is IDLE
            this.robot.intake.setState(Intake.IntakeState.Stopped);
            this.robot.feedStop.engage();
            this.robot.shooter.setState(Shooter.ShooterState.Shooting);
            this.robot.shooter.setTargetRPM(ShooterConstants.Targets.idleRPM);
            this.robot.kicker.setState(Kicker.KickerState.Spinning);
            this.robot.kicker.setTargetRPMFromFlywheel(ShooterConstants.Targets.idleRPM);
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
            this.robot.hood.setAngle(HoodConstants.Limits.hoodHardStopMin);
        }));

        // INTAKE state: left trigger held
        stateTransitionTrigger(ShotCycleState.INTAKE).whenActive(new InstantCommand(() -> {
            // Transitioning to INTAKE state
            this.robot.intake.setState(Intake.IntakeState.Intake);
            this.robot.feedStop.engage();
            // Same as prior state IDLE
            this.robot.shooter.setState(Shooter.ShooterState.Shooting);
            this.robot.shooter.setTargetRPM(ShooterConstants.Targets.idleRPM);
            this.robot.kicker.setState(Kicker.KickerState.Spinning);
            this.robot.kicker.setTargetRPMFromFlywheel(ShooterConstants.Targets.idleRPM);
            // Feed wheel turned on during transition to move game pieces up closer to shooter and within alignment of color sensors.
            // When feedhigh sensor detects gamepiece the feedwheel will be set to stopped in the UpdateState loop
            this.robot.shooterFeed.setFeedPower(this.getTargetFeedPower());
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Feeding);
            this.robot.hood.setAngle(HoodConstants.Limits.hoodHardStopMin);
        }));

        // STAGED: ball is staged but not yet preparing to shoot
        stateTransitionTrigger(ShotCycleState.STAGED).whenActive(new InstantCommand(() -> {
            // Transitioning to STAGED state
            this.robot.intake.setState(Intake.IntakeState.Stopped);
            // Same as prior state INTAKE
            this.robot.feedStop.engage();
            this.robot.shooter.setState(Shooter.ShooterState.Shooting);
            this.robot.shooter.setTargetRPM(ShooterConstants.Targets.idleRPM);
            this.robot.kicker.setState(Kicker.KickerState.Spinning);
            this.robot.kicker.setTargetRPMFromFlywheel(ShooterConstants.Targets.idleRPM);
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
            this.robot.hood.setAngle(HoodConstants.Limits.hoodHardStopMin);
        }));

        // PREPARE_SHOT: spin up, aim hood, keep feedStop engaged
        stateTransitionTrigger(ShotCycleState.PREPARE_SHOT).whenActive(new InstantCommand(() -> {
            // Transitioning to PREPARE_SHOT state
            this.robot.feedStop.engage();
            this.robot.shooter.setState(Shooter.ShooterState.Shooting);
            this.robot.kicker.setState(Kicker.KickerState.Spinning);
            this.robot.hood.setAngle(this.getTargetHoodAngle());
            this.robot.intake.setState(Intake.IntakeState.Stopped);
            // Same as prior state STAGED
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
            // Hood angle set each loop by runPrepareShotLogic() → autoTargetShot()
        }));

        // READY_TO_SHOOT: same hardware as PREPARE_SHOT, just logically "ready"
        stateTransitionTrigger(ShotCycleState.READY_TO_SHOOT).whenActive(new InstantCommand(() -> {
            // Transitioning to READY_TO_SHOOT state
            // Same as prior state PREPARE_SHOT
            this.robot.feedStop.engage();
            this.robot.shooter.setState(Shooter.ShooterState.Shooting);
            this.robot.kicker.setState(Kicker.KickerState.Spinning);
            this.robot.hood.setAngle(this.getTargetHoodAngle());
            this.robot.intake.setState(Intake.IntakeState.Stopped);
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
        }));

        // SHOOTING: open feedStop + run feed & intake while keeping shooter/hood targets
        stateTransitionTrigger(ShotCycleState.SHOOTING).whenActive(new InstantCommand(() -> {
            this.robot.feedStop.disengage();
            this.robot.shooter.setState(Shooter.ShooterState.Shooting);
            this.robot.kicker.setState(Kicker.KickerState.Spinning);
            this.robot.shooterFeed.setFeedPower(this.getTargetFeedPower());
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Feeding);
            this.robot.intake.setState(Intake.IntakeState.Intake);
            // Hood angle set each loop by runPrepareShotLogic() → autoTargetShot()
        }));

        stateTransitionTrigger(ShotCycleState.RECOVERY).whenActive(new InstantCommand(() -> {
            // Entering RECOVERY:
            //  - Keep gate open (feedStop DISENGAGED)
            //  - Stop feed wheel so we aren’t pushing balls
            //  - Keep shooter + hood at their current targets

            this.robot.feedStop.disengage();  // do NOT re-engage, avoids jamming
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
            this.robot.intake.setState(Intake.IntakeState.Stopped);

            this.robot.shooter.setState(Shooter.ShooterState.Shooting);
            this.robot.kicker.setState(Kicker.KickerState.Spinning);
            // Hood angle set each loop by runPrepareShotLogic() → autoTargetShot()
        }));
    }
}