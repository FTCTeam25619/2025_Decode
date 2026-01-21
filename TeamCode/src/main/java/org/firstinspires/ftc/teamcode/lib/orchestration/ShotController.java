package org.firstinspires.ftc.teamcode.lib.orchestration;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.constants.HoodConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterFeedConstants;
import org.firstinspires.ftc.teamcode.lib.triggers.SupplierTrigger;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeed;
import com.seattlesolvers.solverslib.util.MathUtils;


import java.util.function.Predicate;
/**
 * ═══════════════════════════════════════════════════════════════════════════════════════
 * SHOT CONTROLLER - State Machine for Managing Shot Cycles
 * ═══════════════════════════════════════════════════════════════════════════════════════
 *
 * This class orchestrates the complete shot cycle from intake through shooting.
 * It manages shooter RPM, hood angle, feed mechanisms, and auto-targeting.
 *
 * ═══════════════════════════════════════════════════════════════════════════════════════
 * STATE MACHINE FLOWCHART
 * ═══════════════════════════════════════════════════════════════════════════════════════
 *
 * LEGEND:
 *   [STATE]           = Current state
 *   ─────────>        = Automatic transition (no driver input)
 *   ═════════>        = Driver action required
 *   Condition?        = Decision point
 *   {Action}          = What happens during transition
 *
 * ┌───────────────────────────────────────────────────────────────────────────────────┐
 * │                                                                                   │
 * │  STARTING STATE (Auto Mode Only)                                                  │
 * │  ┌───────┐                                                                        │
 * │  │  OFF  │  Used in autonomous - all mechanisms disabled                          │
 * │  └───┬───┘                                                                        │
 * │      │                                                                            │
 * │      │ {Auto starts}                                                              │
 * │      └────────────────────────────────────────────────────────┐                   │
 * │                                                               │                   │
 * │  ┌──────────────────────────────────────────────────────────────────────────────┐ │
 * │  │  DEFAULT STARTING STATE (TeleOp)                                             │ │
 * │  │  ┌──────┐                                                                    │ │
 * │  │  │ IDLE │  Shooter: Idle RPM, Hood: Max angle, FeedStop: Open                │ │
 * │  │  └───┬──┘  Intake: Off, Feed: Off                                            │ │
 * │  │      │                                                                       │ │
 * │  │      │                                                                       │ │
 * │  │      │ ═══════> Driver holds LT (Left Trigger)                               │ │
 * │  │      │          {Turn on intake, close feed stop}                            │ │
 * │  │      ▼                                                                       │ │
 * │  │  ┌────────┐                                                                  │ │
 * │  │  │ INTAKE │  Intake: Running, FeedStop: Closed                               │ │
 * │  │  └────┬───┘  Shooter: Idle RPM, Feed: Off                                    │ │
 * │  │       │                                                                      │ │
 * │  │       │                                                                      │ │
 * │  │       │ ──────> Ball detected by sensor?                                     │ │
 * │  │       │         {Automatically transition}                                   │ │
 * │  │       ▼                                                                      │ │
 * │  │  ┌────────┐                                                                  │ │
 * │  │  │ STAGED │  Ball(s) ready behind feed stop                                  │ │
 * │  │  └────┬───┘  Intake: Off, FeedStop: Closed                                   │ │
 * │  │       │      Shooter: Idle RPM, Feed: Off                                    │ │
 * │  │       │                                                                      │ │
 * │  │       │                                                                      │ │
 * │  │       │ ═══════> Driver holds LB (Left Bumper)                               │ │
 * │  │       │          {Spin up shooter, aim hood}                                 │ │
 * │  │       ▼                                                                      │ │
 * │  │  ┌──────────────┐                                                            │ │
 * │  │  │ PREPARE_SHOT │  Shooter: Spinning up to target RPM                        │ │
 * │  │  └──────┬───────┘  Hood: Moving to target angle                              │ │
 * │  │         │            Robot: Auto-aiming to target (if enabled)               │ │
 * │  │         │            FeedStop: Closed, Feed: Off                             │ │
 * │  │         │                                                                    │ │
 * │  │         │            ┌─────────────────────────────────────┐                 │ │
 * │  │         │            │ TARGETING MODE (Light 4 indicator)  │                 │ │
 * │  │         │            │                                     │                 │ │
 * │  │         │            │ GREEN: Auto-shoot (LUT)             │                 │ │
 * │  │         │            │  - Uses distance to calculate RPM   │                 │ │
 * │  │         │            │  - Updates continuously             │                 │ │
 * │  │         │            │                                     │                 │ │
 * │  │         │            │ RED: Preset mode                    │                 │ │
 * │  │         │            │  - Fixed RPM/angle from preset      │                 │ │
 * │  │         │            │  - No auto-calculation              │                 │ │
 * │  │         │            └─────────────────────────────────────┘                 │ │
 * │  │         │                                                                    │ │
 * │  │         │ ──────> All conditions met? ┐                                      │ │
 * │  │         │         - Shooter at RPM     │                                     │ │
 * │  │         │         - Hood at angle      │                                     │ │
 * │  │         │         - Robot at heading   │                                     │ │
 * │  │         │         {Light 2 turns GREEN}│                                     │ │
 * │  │         ▼                               │                                    │ │
 * │  │  ┌────────────────┐                    │                                     │ │
 * │  │  │ READY_TO_SHOOT │  All conditions met, waiting for shoot command           │ │
 * │  │  └────────┬───────┘  Lights: Green = Ready                                   │ │
 * │  │           │            FeedStop: Still closed                                │ │
 * │  │           │                                                                  │ │
 * │  │           │ ◄──────────────────────────┘ Not ready anymore?                  │ │
 * │  │           │                              {Fall back to PREPARE_SHOT}         │ │
 * │  │           │                                                                  │ │
 * │  │           │                                                                  │ │
 * │  │           │ ═══════> Driver pulls RT (Right Trigger)                         │ │
 * │  │           │          {Open feed stop, run feed + intake}                     │ │
 * │  │           ▼                                                                  │ │
 * │  │  ┌──────────┐                                                                │ │
 * │  │  │ SHOOTING │  FeedStop: OPEN                                                │ │
 * │  │  └─────┬────┘  Feed: Running, Intake: Running                                │ │
 * │  │        │       Shooter: At target RPM                                        │ │
 * │  │        │       Balls feeding through shooter                                 │ │
 * │  │        │                                                                     │ │
 * │  │        │                                                                     │ │
 * │  │        │ ──────> RPM drops? ┐                                                │ │
 * │  │        │         {Temporary disturbance from shooting}                       │ │
 * │  │        ▼                     │                                               │ │
 * │  │  ┌──────────┐               │                                                │ │
 * │  │  │ RECOVERY │  FeedStop: OPEN (stays open!)                                  │ │
 * │  │  └─────┬────┘  Feed: STOPPED (prevent jamming)                               │ │
 * │  │        │       Intake: Stopped                                               │ │
 * │  │        │       Shooter: Recovering to target RPM                             │ │
 * │  │        │                                                                     │ │
 * │  │        │ ◄─────────────────┘ RPM recovered?                                  │ │
 * │  │        │                     {Resume SHOOTING}                               │ │
 * │  │        │                                                                     │ │
 * │  │        │ ═══════> Driver releases RT                                         │ │
 * │  │        │          OR no more balls detected                                  │ │
 * │  │        │          {Return to appropriate state}                              │ │
 * │  │        │                                                                     │ │
 * │  │        └────────> Still holding LB? ─Yes─> Back to PREPARE_SHOT              │ │
 * │  │                                      │                                       │ │
 * │  │                                      └─No──> Back to IDLE ◄──────────────────┤ │
 * │  │                                                                              │ │
 * │  └──────────────────────────────────────────────────────────────────────────────┘ │
 * │                                                                                   │
 * └───────────────────────────────────────────────────────────────────────────────────┘
 *
 * ═════════════════════════════════════════════════════════════════════════════════════
 * STATE TRANSITION TRIGGER SUMMARY
 * ═════════════════════════════════════════════════════════════════════════════════════
 *
 * MANUAL TRANSITIONS (Driver Input):
 * ┌───────────────────┬─────────────────┬──────────────────────────────────────────────┐
 * │ From State        │ Input           │ To State                                     │
 * ├───────────────────┼─────────────────┼──────────────────────────────────────────────┤
 * │ IDLE              │ LT pressed      │ INTAKE                                       │
 * │ INTAKE            │ LT released     │ IDLE (or auto to STAGED if ball detected)    │
 * │ STAGED            │ LB pressed      │ PREPARE_SHOT                                 │
 * │ PREPARE_SHOT      │ LB released     │ IDLE (if RT also released)                   │
 * │ READY_TO_SHOOT    │ RT pressed      │ SHOOTING                                     │
 * │ SHOOTING/RECOVERY │ RT released     │ PREPARE_SHOT (if LB held) or IDLE            │
 * │ Any State         │ DPad Up         │ IDLE (emergency)                             │
 * │ Any State         │ DPad Down       │ OFF (shutdown)                               │
 * └───────────────────┴─────────────────┴──────────────────────────────────────────────┘
 *
 * AUTOMATIC TRANSITIONS (No Driver Input):
 * ┌───────────────────┬─────────────────────────────────┬────────────────────────────┐
 * │ From State        │ Condition                       │ To State                   │
 * ├───────────────────┼─────────────────────────────────┼────────────────────────────┤
 * │ INTAKE            │ Ball detected by sensor         │ STAGED                     │
 * │ PREPARE_SHOT      │ All ready conditions met        │ READY_TO_SHOOT             │
 * │ READY_TO_SHOOT    │ Any ready condition not met     │ PREPARE_SHOT               │
 * │ SHOOTING          │ RPM drops below tolerance       │ RECOVERY                   │
 * │ RECOVERY          │ RPM back at target              │ SHOOTING                   │
 * └───────────────────┴─────────────────────────────────┴────────────────────────────┘
 *
 * ═══════════════════════════════════════════════════════════════════════════════════════
 * MECHANISM STATE BY SHOT CONTROLLER STATE
 * ═══════════════════════════════════════════════════════════════════════════════════════
 *
 * ┌───────────────┬──────────┬──────────┬─────────┬────────┬─────────────────────────┐
 * │ State         │ Intake   │ FeedStop │ Shooter │ Feed   │ Hood                    │
 * ├───────────────┼──────────┼──────────┼─────────┼────────┼─────────────────────────┤
 * │ OFF           │ Stopped  │ Open     │ Off     │ Off    │ Max Angle               │
 * │ IDLE          │ Stopped  │ Open     │ Idle    │ Off    │ Max Angle               │
 * │ INTAKE        │ Running  │ CLOSED   │ Idle    │ Off    │ Max Angle               │
 * │ STAGED        │ Stopped  │ CLOSED   │ Idle    │ Off    │ Max Angle               │
 * │ PREPARE_SHOT  │ Stopped  │ CLOSED   │ Spin Up │ Off    │ Moving to Target        │
 * │ READY_TO_SHOOT│ Stopped  │ CLOSED   │ At RPM  │ Off    │ At Target               │
 * │ SHOOTING      │ Running  │ OPEN     │ At RPM  │ RUNNING│ At Target               │
 * │ RECOVERY      │ Stopped  │ OPEN     │ Recover │ STOPPED│ At Target               │
 * └───────────────┴──────────┴──────────┴─────────┴────────┴─────────────────────────┘
 *
 * ═══════════════════════════════════════════════════════════════════════════════════════
 * INDICATOR LIGHTS DURING SHOOTING STATES
 * ═══════════════════════════════════════════════════════════════════════════════════════
 *
 * When isShooting() returns true (PREPARE_SHOT, READY_TO_SHOOT, SHOOTING, RECOVERY):
 *
 * ┌────────┬───────────────────────────────────────┬────────────────────────────────┐
 * │ Light  │ Shows                                 │ Color Logic                    │
 * ├────────┼───────────────────────────────────────┼────────────────────────────────┤
 * │ Light 1│ Shooter RPM Status                    │ GREEN = at target RPM          │
 * │        │                                       │ RED = spinning up/recovering   │
 * ├────────┼───────────────────────────────────────┼────────────────────────────────┤
 * │ Light 2│ Overall Ready Status                  │ GREEN = ready to shoot         │
 * │        │ (RPM + Hood + Heading all good)       │ RED = not ready                │
 * ├────────┼───────────────────────────────────────┼────────────────────────────────┤
 * │ Light 3│ Heading Alignment                     │ GREEN = aimed at target        │
 * │        │                                       │ RED = still turning            │
 * ├────────┼───────────────────────────────────────┼────────────────────────────────┤
 * │ Light 4│ Targeting Mode (Independent)          │ GREEN = Auto-shoot (LUT)       │
 * │        │                                       │ RED = Preset shot              │
 * └────────┴───────────────────────────────────────┴────────────────────────────────┘
 *
 * When NOT shooting (IDLE, INTAKE, STAGED):
 *   Lights 1-3 show ball detection (sensor colors)
 *   Light 4 still shows targeting mode
 *
 * ═══════════════════════════════════════════════════════════════════════════════════════
 * READY TO SHOOT CONDITIONS
 * ═══════════════════════════════════════════════════════════════════════════════════════
 *
 * For readyToShoot() to return true, ALL must be satisfied:
 *
 *   ✓ Shooter at target RPM (within tolerance)
 *   ✓ Hood at target angle (within tolerance)
 *   ✓ Robot at target heading (within tolerance)
 *
 * Missing any condition → Light 2 RED, cannot shoot
 * All conditions met → Light 2 GREEN, ready to shoot
 *
 * ═══════════════════════════════════════════════════════════════════════════════════════
 * TARGETING MODES
 * ═══════════════════════════════════════════════════════════════════════════════════════
 *
 * AUTO-SHOOT MODE (presetShotMode = false, Light 4 GREEN):
 *   - Uses LUT (Lookup Table) based on distance to target
 *   - Distance measured from Limelight (preferred) or odometry (backup)
 *   - RPM and hood angle calculated automatically
 *   - Updates continuously as robot moves
 *   - Activated by: DPad Left or starting state
 *
 * PRESET SHOT MODE (presetShotMode = true, Light 4 RED):
 *   - Uses fixed RPM, hood angle, feed power from PresetShot enum
 *   - No auto-calculation, values don't change
 *   - Good for: consistent shots from known positions
 *   - Activated by: Back + A/X/B/Y (selects preset)
 *   - Available presets:
 *     • AtGoal - Right at the goal
 *     • NearZoneMid - Middle of near scoring zone
 *     • NearZoneEnd - Far edge of near zone
 *     • FarWall - From far wall position
 *
 * ═══════════════════════════════════════════════════════════════════════════════════════
 * RECOVERY STATE EXPLAINED
 * ═══════════════════════════════════════════════════════════════════════════════════════
 *
 * WHY RECOVERY EXISTS:
 *   When shooting, balls exiting the flywheel temporarily slow it down (mechanical load).
 *   If we keep feeding while RPM is low, balls won't shoot properly and may jam.
 *
 * WHAT RECOVERY DOES:
 *   1. Detects RPM drop during SHOOTING
 *   2. STOPS the feed wheel (prevents more balls entering)
 *   3. Keeps FeedStop OPEN (important - prevents jamming)
 *   4. Lets shooter recover to target RPM
 *   5. Automatically returns to SHOOTING when RPM recovers
 *
 * KEY DESIGN DECISION:
 *   FeedStop stays OPEN during recovery to avoid re-engaging on a partially-fed ball.
 *   This prevents jamming at the cost of potentially losing a ball if driver releases RT.
 *
 * ═══════════════════════════════════════════════════════════════════════════════════════
 */
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

    private final InterpLUT shooterRPMInterpolator;
    private final InterpLUT hoodAngleInterpolator;
    private static final double MIN_LUT_DISTANCE_CM = 0.0;
    private static final double MAX_LUT_DISTANCE_CM = 600.0;


    public ShotController(NovaPyraRobot robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;

        this.shooterRPMInterpolator = setupShooterRPMInterpolator();
        this.hoodAngleInterpolator = setupHoodAngleInterpolator();
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

        telemetry.addData("ShotController: Prior State", this.priorState.name());
        telemetry.addData("ShotController: Current State", this.currentState.name());
        telemetry.addData("ShotController: Transitioning?", this.stateTransition);
        telemetry.addData("ShotController: RPM Target", this.getTargetFlywheelRPM());
        telemetry.addData("ShotController: Hood Target", this.getTargetHoodAngle());
        telemetry.addData("ShotController: Feed Power Target", this.getTargetFeedPower());
        telemetry.addData("ShotController: Preset Mode?", this.presetShotMode);

        this.priorState = this.currentState;

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
                // While shooting, if we fall out of "ready" (RPM drop, hood/heading error), stop feeding and go into recovery.
                if (!this.isRPMatTarget()) {
                    this.currentState = ShotCycleState.RECOVERY;
                }
                break;

            case RECOVERY:
                // In recovery we keep shooter + hood at target, but feed is stopped.
                // Once we are “ready” again, resume SHOOTING.
                if (this.isRPMatTarget()) {
                    this.currentState = ShotCycleState.SHOOTING;
                }
                break;

        }
    }

    /**
     * Shared logic for PREPARE_SHOT and READY_TO_SHOOT.
     * - Auto-target RPM/hood if we're not in preset mode.
     * - Apply RPM/hood targets every loop so changes propagate quickly.
     * - Heading alignment control happens in NovaPyraRobot.run using vision/odometry.
     */
    private void runPrepareShotLogic() {
        if (!presetShotMode) {
            autoTargetShot();
        }

        robot.shooter.setTargetRPM(this.getTargetFlywheelRPM());
        robot.hood.setAngle(this.getTargetHoodAngle());
        // Feed stays stopped until SHOOTING state.
    }

    // --------------------------
    // LUT Targeting
    // --------------------------

    public void autoTargetShot() {
        this.presetShotMode = false;

        double rawDistance = getDistanceToTarget();

        // If we don't have a valid distance (no tag, NaN, etc.), don't touch current targets
        if (Double.isNaN(rawDistance) || Double.isInfinite(rawDistance)) {
            telemetry.addData("Autoshoot", "No valid distance (no vision target); skipping LUT");
            return;
        }

        // Clamp to valid LUT range to avoid tiny floating point out-of-bounds
        double distance = MathUtils.clamp(rawDistance, MIN_LUT_DISTANCE_CM, MAX_LUT_DISTANCE_CM);

        setTargetFlywheelRPM(shooterRPMInterpolator.get(distance));
        setTargetHoodAngle(hoodAngleInterpolator.get(distance));
        // setTargetFeedPower(feedPowerInterpolator.get(distance));

        telemetry.addData("Autoshoot Raw Distance (cm)", rawDistance);
        telemetry.addData("Autoshoot LUT Distance (cm)", distance);
    }


    /**
     * Hybrid distance:
     * - If Limelight sees tag, use LL distance (Vision.distanceToTarget()).
     * - Otherwise, use odometry distance to a known goal point on the field.
     */
    private double getDistanceToTarget() {
        if (robot.vision.isSeeingTarget()) {
            // Limelight reports distance in cm.
            return robot.vision.distanceToTarget();
        }

        // No valid distance when we don't see the tag
        return Double.NaN;
    }


    public void setupPresetShot(PresetShot shotSetting) {
        this.presetShotMode    = true;
        this.targetFlywheelRPM = shotSetting.targetFlywheelRPM;
        this.targetHoodAngle   = shotSetting.targetHoodAngle;
        this.targetFeedPower   = shotSetting.targetFeedPower;
        //this.targetFlywheelRPM = ShooterConstants.Targets.atGoalShot;
        //this.targetHoodAngle = HoodConstants.Targets.atGoalShot;

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
        return this.robot.shooter.atRPM();
    }

    private boolean atTargetAngle() {
        return this.robot.hood.atTargetAngle();
    }

    private boolean atTargetHeading() {
        // Implement hybrid LL/odometry logic inside Vision.atHeading()
        return this.robot.vision.atHeading();
    }

    public boolean readyToShoot() {
        return this.atRPM() && this.atTargetAngle() && this.atTargetHeading();
    }

    public boolean isRPMatTarget(){
        return this.atRPM();
    }

    public boolean transitionedTo(ShotCycleState newState) {
        return this.stateTransition && this.currentState == newState;
    }

    // --------------------------
    // Transition side-effects
    // --------------------------

    public void setupStateTransitionTriggers() {
        stateTransitionTrigger(ShotCycleState.OFF).whenActive(new InstantCommand(() -> {
            // Transitioning back to OFF state
            this.robot.intake.setState(Intake.IntakeState.Stopped);
            this.robot.feedStop.disengage();
            this.robot.shooter.setState(Shooter.ShooterState.Stopped);
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
            this.robot.hood.setAngle(HoodConstants.Limits.hoodHardStopMax);
        }));

        stateTransitionTrigger(ShotCycleState.IDLE).whenActive(new InstantCommand(() -> {
            // Transitioning back to IDLE state
            // NOTE: This will not trigger at start because initial state is IDLE
            this.robot.intake.setState(Intake.IntakeState.Stopped);
            this.robot.feedStop.disengage();
            this.robot.shooter.setState(Shooter.ShooterState.Shooting);
            this.robot.shooter.setTargetRPM(ShooterConstants.Targets.idleRPM);
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
            this.robot.hood.setAngle(HoodConstants.Limits.hoodHardStopMax);
        }));

        // INTAKE state: left trigger held
        stateTransitionTrigger(ShotCycleState.INTAKE).whenActive(new InstantCommand(() -> {
            // Transitioning to INTAKE state
            this.robot.intake.setState(Intake.IntakeState.Intake);
            this.robot.feedStop.engage();
            // Same as prior state IDLE
            this.robot.shooter.setState(Shooter.ShooterState.Shooting);
            this.robot.shooter.setTargetRPM(ShooterConstants.Targets.idleRPM);
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
            this.robot.hood.setAngle(HoodConstants.Limits.hoodHardStopMax);
        }));

        // STAGED: ball is staged but not yet preparing to shoot
        stateTransitionTrigger(ShotCycleState.STAGED).whenActive(new InstantCommand(() -> {
            // Transitioning to STAGED state
            this.robot.intake.setState(Intake.IntakeState.Stopped);
            // Same as prior state INTAKE
            this.robot.feedStop.engage();
            this.robot.shooter.setState(Shooter.ShooterState.Shooting);
            this.robot.shooter.setTargetRPM(ShooterConstants.Targets.idleRPM);
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
            this.robot.hood.setAngle(HoodConstants.Limits.hoodHardStopMax);
        }));

        // PREPARE_SHOT: spin up, aim hood, keep feedStop engaged
        stateTransitionTrigger(ShotCycleState.PREPARE_SHOT).whenActive(new InstantCommand(() -> {
            // Transitioning to PREPARE_SHOT state
            this.robot.feedStop.engage();
            this.robot.shooter.setState(Shooter.ShooterState.Shooting);
            this.robot.shooter.setTargetRPM(this.getTargetFlywheelRPM());
            this.robot.hood.setAngle(this.getTargetHoodAngle());
            this.robot.intake.setState(Intake.IntakeState.Stopped);
            // Same as prior state STAGED
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
        }));

        // READY_TO_SHOOT: same hardware as PREPARE_SHOT, just logically "ready"
        stateTransitionTrigger(ShotCycleState.READY_TO_SHOOT).whenActive(new InstantCommand(() -> {
            // Transitioning to READY_TO_SHOOT state
            // Same as prior state PREPARE_SHOT
            this.robot.feedStop.engage();
            this.robot.shooter.setState(Shooter.ShooterState.Shooting);
            this.robot.shooter.setTargetRPM(this.getTargetFlywheelRPM());
            this.robot.hood.setAngle(this.getTargetHoodAngle());
            this.robot.intake.setState(Intake.IntakeState.Stopped);
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
        }));

        // SHOOTING: open feedStop + run feed & intake while keeping shooter/hood targets
        stateTransitionTrigger(ShotCycleState.SHOOTING).whenActive(new InstantCommand(() -> {
            this.robot.feedStop.disengage();
            this.robot.shooter.setState(Shooter.ShooterState.Shooting);
            this.robot.shooterFeed.setFeedPower(this.getTargetFeedPower());
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Feeding);
            this.robot.intake.setState(Intake.IntakeState.Intake);
            this.robot.shooter.setTargetRPM(this.getTargetFlywheelRPM());
            this.robot.hood.setAngle(this.getTargetHoodAngle());
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
            this.robot.shooter.setTargetRPM(this.getTargetFlywheelRPM());
            this.robot.hood.setAngle(this.getTargetHoodAngle());
        }));

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

    public Trigger autoShootMode() {
        if (this._presetShotModeTrigger == null) {
            this._presetShotModeTrigger = new SupplierTrigger(() -> !this.presetShotMode);
        }
        return this._presetShotModeTrigger.negate();
    }

    public Trigger presetShotMode() {
        if (this._presetShotModeTrigger == null) {
            this._presetShotModeTrigger = new SupplierTrigger(() -> this.presetShotMode);
        }
        return this._presetShotModeTrigger;
    }

    public boolean doAutoHeading() {
        return this.currentState == ShotCycleState.PREPARE_SHOT ||
                this.currentState == ShotCycleState.READY_TO_SHOOT;
    }

    public boolean isShooting() {
        return this.currentState == ShotCycleState.PREPARE_SHOT ||
                this.currentState == ShotCycleState.READY_TO_SHOOT ||
                this.currentState == ShotCycleState.SHOOTING ||
                this.currentState == ShotCycleState.RECOVERY;
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
        ShotCycleState checkState;
        Predicate<ShotCycleState> getFn;
        public StateTransitionToTrigger(Predicate<ShotCycleState> shotCycleStateSupplier, ShotCycleState state) {
            this.checkState = state;
            this.getFn = shotCycleStateSupplier;
        }

        @Override
        public boolean get() { return this.getFn.test(checkState); }
    }

    // --------------------------
    // LUT setup
    // --------------------------

    private InterpLUT setupShooterRPMInterpolator() {
        InterpLUT table = new InterpLUT();
       // table.add(OdometryConstants.TargetDistances.atGoalShot,      ShooterConstants.RPMTargets.atGoalShot);
       // table.add(OdometryConstants.TargetDistances.nearZoneMidShot, ShooterConstants.RPMTargets.nearZoneMidShot);
       // table.add(OdometryConstants.TargetDistances.nearZoneEndShot, ShooterConstants.RPMTargets.nearZoneEndShot);
       // table.add(OdometryConstants.TargetDistances.farWallShot,     ShooterConstants.RPMTargets.farWallShot);


        table.add(0, 1850);

        // Big Triangle / Close Shots
        table.add(51, 1950);
        table.add(69, 2050);
        table.add(109, 2300);
        table.add(129, 2350);
        table.add(152, 2500);
        table.add(172, 2500);
        table.add(177, 2500);
        table.add(190, 2500);
        table.add(212, 2650);

        // Small Triangle / Far Shots
        table.add(280, 2850);
        table.add(290, 2900);
        table.add(310, 2950);
        table.add(330, 3050);
        table.add(354, 3100);


        table.add(600, 3200);
        table.createLUT();
        return table;
    }

    private InterpLUT setupHoodAngleInterpolator() {
        InterpLUT table = new InterpLUT();
        //table.add(OdometryConstants.TargetDistances.atGoalShot,      HoodConstants.Targets.atGoalShot);
        //table.add(OdometryConstants.TargetDistances.nearZoneMidShot, HoodConstants.Targets.nearZoneMidShot);
        //table.add(OdometryConstants.TargetDistances.nearZoneEndShot, HoodConstants.Targets.nearZoneEndShot);
        //table.add(OdometryConstants.TargetDistances.farWallShot,     HoodConstants.Targets.farWallShot);


        table.add(0, 275);

        // Big Triangle / Close Shots
        table.add(51, 275);
        table.add(69, 275);
        table.add(109, 230);
        table.add(129, 210);
        table.add(152, 200);
        table.add(172, 180);
        table.add(177, 165);
        table.add(190, 110);
        table.add(212, 110);

        // Small Triangle / Far Shots
        table.add(280, 10);
        table.add(290, 10);
        table.add(310, 10);
        // FIXME: Soft stop on servo currently set at 10. Do we really need to go below that?
        table.add(330, 0);
        table.add(354, 0);

        table.add(600, 0);

        table.createLUT();
        return table;
    }
}