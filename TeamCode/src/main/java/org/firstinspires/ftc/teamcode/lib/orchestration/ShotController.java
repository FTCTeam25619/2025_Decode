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
                if (!canMakeShot()) {
                    this.currentState = ShotCycleState.RECOVERY;
                }
                break;

            case RECOVERY:
                runPrepareShotLogic();
                if (canMakeShot()) {
                    this.currentState = ShotCycleState.SHOOTING;
                }
                break;
        }
    }

    /**
     * Shared logic for PREPARE_SHOT, READY_TO_SHOOT, SHOOTING, and RECOVERY.
     * Calls autoTargetShot() every loop unless in preset mode.
     */
    private void runPrepareShotLogic() {
        if (!presetShotMode) {
            autoTargetShot();
        }


        robot.shooter.setTargetRPM(this.getTargetFlywheelRPM());
        if (presetShotMode) {
            robot.hood.setAngle(this.getTargetHoodAngle());
        }
    }

    // --------------------------
    // Physics Solver
    // --------------------------

    /**
     * Main solver entry point.
     *
     * Strategy:
     *   - Commands baseline RPM from LUT.
     *   - Reads ACTUAL RPM (may be less than baseline due to drop between shots).
     *   - Uses flatFirstSolveHoodAngle() to find the shallowest launch angle that
     *     puts the ball's arc through the goal center at the current distance,
     *     given actual v0. Prefers launchAngleMin, steepens only as needed.
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

        double baseline = baselineRPMLUT.get(distanceIn) + ShooterConstants.Solver.baselineOffset;
        setTargetFlywheelRPM(baseline);

        double actualRPM      = robot.shooter.getFlywheelRPM();
        double launchAngleDeg = flatFirstSolveHoodAngle(distanceIn, actualRPM);

        robot.hood.setLaunchAngle(launchAngleDeg);

        telemetry.addData("Autoshoot Distance (in)",    distanceIn);
        telemetry.addData("Autoshoot Baseline RPM",     baseline);
        telemetry.addData("Autoshoot Actual RPM",       actualRPM);
        telemetry.addData("Autoshoot Launch Angle (°)", launchAngleDeg);
        telemetry.addData("Autoshoot Can Make Shot",    canMakeShot(distanceIn, actualRPM));
    }

    /**
     * Flat-first hood angle solver.
     *
     * Finds the shallowest launch angle within [launchAngleMin, launchAngleMax]
     * whose arc passes through the goal center at the given distance and RPM.
     *
     * Algorithm:
     *   1. Evaluate ball height at angleMin and angleMax.
     *   2. If angleMin already hits within HIT_TOLERANCE → return angleMin (flattest possible).
     *   3. If heights straddle goalCenterHeight → bisect to find the crossing angle,
     *      then step back toward angleMin to find the flattest angle still within tolerance.
     *   4. If no crossing exists → return whichever boundary is closest.
     *      canMakeShot() will catch misses and trigger RECOVERY.
     *
     * Returns a clamped angle — always safe to command.
     */
    private double flatFirstSolveHoodAngle(double distanceIn, double actualRPM) {
        final double angleMin = HoodConstants.Limits.launchAngleMin;
        final double angleMax = maxAngleForDistance(distanceIn);

        double hMin = ballHeightAtGoal(angleMin, distanceIn, actualRPM);
        double hMax = ballHeightAtGoal(angleMax, distanceIn, actualRPM);

        // Case 1: flat angle already hits descending — use it as-is
        if (!Double.isNaN(hMin) && hMin > 0
                && Math.abs(hMin - ShooterConstants.Solver.goalCenterHeight)
                <= ShooterConstants.Solver.hitTolerance) {
            return angleMin;
        }

        // Case 2: even max angle can't produce a descending valid shot — return max and let
        // canMakeShot() trigger RECOVERY. Nothing safe to do here.
        if (Double.isNaN(hMax) || hMax <= 0) {
            return angleMax;
        }

        // Case 3: bisect. NaN means ascending (not steep enough) — treat as lo side.
        double lo = angleMin, hi = angleMax;
        double mid = angleMin;
        for (int i = 0; i < 40; i++) {
            mid = (lo + hi) / 2.0;
            double hMid = ballHeightAtGoal(mid, distanceIn, actualRPM);
            if (!Double.isNaN(hMid)
                    && Math.abs(hMid - ShooterConstants.Solver.goalCenterHeight) < 0.05) break;
            // NaN = ascending = not steep enough → push lo up toward steeper angles
            if (Double.isNaN(hMid) || hMid > ShooterConstants.Solver.goalCenterHeight) {
                lo = mid;
            } else {
                hi = mid;
            }
        }

        // Step back toward angleMin — flattest angle still producing descending valid shot
        double bestAngle = mid;
        for (double a = mid - 0.1; a >= angleMin; a -= 0.1) {
            double h = ballHeightAtGoal(a, distanceIn, actualRPM);
            if (!Double.isNaN(h) && h > 0
                    && Math.abs(h - ShooterConstants.Solver.goalCenterHeight)
                    <= ShooterConstants.Solver.hitTolerance) {
                bestAngle = a;
            } else {
                break;
            }
        }

        return MathUtils.clamp(bestAngle, angleMin, angleMax);
    }

    /**
     * Returns the ball's height (inches) when it reaches goal x distance,
     * given launch angle (degrees) and actual flywheel RPM.
     *
     * Uses projectile kinematics from the launch point:
     *   y(t) = launchHeight + v0y*t - 0.5*G*t²
     *   x(t) = v0x*t  →  t = distanceIn / v0x
     *
     * Returns a negative value if the ball has already hit the ground before
     * reaching goal x (used by canMakeShot() to detect short shots).
     */
    private double ballHeightAtGoal(double angleDeg, double distanceIn, double actualRPM) {
        double v0       = rpmToV0(actualRPM, angleDeg, distanceIn);
        double angleRad = Math.toRadians(angleDeg);
        double v0x      = v0 * Math.cos(angleRad);
        double v0y      = v0 * Math.sin(angleRad);
        double t        = distanceIn / v0x;
        double ballH    = robot.hood.interpolateLaunchHeight()
                + v0y * t
                - 0.5 * ShooterConstants.Solver.G * t * t;
        double vy       = v0y - ShooterConstants.Solver.G * t;
        // Return NaN if ascending — ascending hits are not valid shots
        return vy < 0 ? ballH : Double.NaN;
    }

    /**
     * Returns true if the current distance + actual RPM produce a valid shot —
     * meaning the ball physically arrives within HIT_TOLERANCE of the goal center
     * at the commanded launch angle.
     *
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
        double currentAngleDeg = robot.hood.getCurrentLaunchAngle();
        double ballH           = ballHeightAtGoal(currentAngleDeg, distanceIn, actualRPM);
        return ballH > 0
                && Math.abs(ballH - ShooterConstants.Solver.goalCenterHeight)
                <= ShooterConstants.Solver.hitTolerance;
    }

    /**
     * Converts flywheel RPM to estimated launch velocity v0 (in/s).
     *
     * v0 = (slope(launchAngle) * RPM + intercept(launchAngle)) * correction(distance)
     *
     * slope/intercept are per-angle calibration LUTs.
     * correction is a distance-keyed multiplier — tune empirically after goal shots.
     */
    private double rpmToV0(double rpm, double angleDeg, double distanceIn) {
        double slope      = v0SlopeLUT.get(angleDeg);
        double intercept  = v0InterceptLUT.get(angleDeg);
        double v0         = (slope * rpm + intercept) * ShooterConstants.Solver.correctionMultiplier;
        double correction = v0CorrectionLUT.get(
                MathUtils.clamp(distanceIn,
                        ShooterConstants.Solver.minDistanceIn,
                        ShooterConstants.Solver.maxDistanceIn));
        return v0 * correction;
    }

    /**
     * Returns y — vertical distance from launch point to goal center (inches).
     * Not used directly by flatFirstSolveHoodAngle (which calls ballHeightAtGoal
     * directly), but retained for any external callers that need the target height.
     */
    private double launchHeightToGoal() {
        return ShooterConstants.Solver.goalCenterHeight
                - robot.hood.interpolateLaunchHeight();
    }

    /**
     * Returns the max hood angle allowed at a given distance.
     * Close shots need a steeper ceiling to get a descending arc into the goal.
     * Far shots are fine at 58° — they already have a large descending window.
     */
    private double maxAngleForDistance(double distanceIn) {
        if (distanceIn <= 40)  return 65.0;   // 35" zone: needs steep to arc over and drop in
        if (distanceIn <= 60)  return 60.0;   // 50" zone: small buffer above 58
        return HoodConstants.Limits.launchAngleMax;  // 58° for everything else
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
        lut.add(0,   1.0717);
        lut.add(35,  1.0717);
        lut.add(80,  1.0717);  // confirmed: 2300 RPM @ 80"
        lut.add(100, 1.0716);  // confirmed: 2500 RPM @ 100"
        lut.add(127, 1.0495);  // confirmed: 2830 RPM @ 127"
        lut.add(133, 1.0495);  // confirmed: 2910 RPM @ 133"
        lut.add(140, 1.0495);  // confirmed: 2980 RPM @ 140"
        lut.add(200, 1.0495);
        lut.createLUT();
        return lut;
    }

    /**
     * Baseline RPM LUT — distance-keyed.
     * Set above the minimum RPM needed at each distance so the solver has
     * headroom to flatten when RPM drops between shots.
     *
     * Tuning guide: If hood angle is too steep → raise baseline at that distance.
     *               If spin-up takes too long  → lower baseline at that distance.
     */
    private InterpLUT setupBaselineRPMLUT() {
        InterpLUT lut = new InterpLUT();
        lut.add(0,   2180);  // 35" close shot — uses steep angle (62-65°)
        lut.add(35,  2180);
        lut.add(50,  2100);
        lut.add(80,  2320);  // real shot confirmed at 2300
        lut.add(100, 2520);  // real shot confirmed at 2500
        lut.add(120, 2890);  // min=2744, +150 headroom
        lut.add(140, 3130);  // min=2980 (confirmed real shot), +150 headroom
        lut.add(160, 3320);  // min=3167, +150 headroom
        lut.add(200, 3320);
        lut.createLUT();
        return lut;
    }
    /**
     * v0 slope LUT — launch angle keyed (degrees).
     * Captures how RPM→v0 sensitivity changes with hood position
     * due to varying ball wrap around the flywheel.
     *
     * TODO: Replace placeholder values with real calibration data
     *       from ShooterCalibration.xlsx Curve Fit sheet.
     */
    private InterpLUT setupV0SlopeLUT() {
        InterpLUT lut = new InterpLUT();

        lut.add(0,     0.0735);
        lut.add(48.25, 0.0735);  // placeholder
        lut.add(50.5,  0.0776);  // placeholder
        lut.add(53.0,  0.0768);  // placeholder
        lut.add(55.5,  0.0724);  // placeholder
        lut.add(58.0,  0.0748);  // placeholder
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

        lut.add(0,     16.7);
        lut.add(48.25, 26.0);  // placeholder
        lut.add(50.5,  15.5);  // placeholder
        lut.add(53.0,  15.5);  // placeholder
        lut.add(55.5,  26.6);  // placeholder
        lut.add(58.0,  14.6);  // placeholder
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

        if (targetPose == null) return false;

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
        return this.atRPM();
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