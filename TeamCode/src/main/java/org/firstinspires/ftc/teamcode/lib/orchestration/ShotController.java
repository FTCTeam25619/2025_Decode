package org.firstinspires.ftc.teamcode.lib.orchestration;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.Devices;
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
                // While shooting, if we fall out of "ready" (RPM drop, hood/heading error), stop feeding and go into recovery.
                if (!this.isRPMatTarget()) {
                    this.currentState = ShotCycleState.RECOVERY;
                }
                break;

            case RECOVERY:
                runPrepareShotLogic();
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

        setTargetFlywheelRPM(shooterRPMInterpolator.get(distance) + ShooterConstants.baselineRPM);
        setTargetHoodAngle(hoodAngleInterpolator.get(distance));

        telemetry.addData("Autoshoot Raw Distance (cm)", rawDistance);
        telemetry.addData("Autoshoot LUT Distance (cm)", distance);
    }


    /**
     * Hybrid distance:
     * - If Limelight sees tag, use LL distance (Vision.distanceToTarget()).
     * - Otherwise, use odometry distance to a known goal point on the field.
     */
    public double getDistanceToTarget() {
        //if (robot.vision.isSeeingTarget()) {
           // Limelight reports distance in cm.
         // return robot.vision.distanceToTarget();
        //}

        // Odometry/Pedro fallback (field distance): poses are in inches, so convert to cm
        Pose robotPose  = robot.pedroFollower.getPose();
        Pose targetPose = robot.vision.getTargetTagPose();

        // Check if we have valid target pose data
        if (targetPose == null) {
            // No valid distance when we don't see the tag
            return Double.NaN;
        }

        double dxIn = targetPose.getX() - robotPose.getX();
        double dyIn = targetPose.getY() - robotPose.getY();

        double distanceIn = Math.hypot(dxIn, dyIn);
        return distanceIn * 2.54; // inches -> cm
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
            this.robot.feedStop.engage();
            this.robot.shooter.setState(Shooter.ShooterState.Stopped);
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
            // Feed wheel turned on during transition to move game pieces up closer to shooter and within alignment of color sensors.
            // When feedhigh sensor detects gamepiece the feedwheel will be set to stopped in the UpdateState loop
            this.robot.shooterFeed.setFeedPower(ShooterFeedConstants.Targets.intakeFeedPower);
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
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
            this.robot.hood.setAngle(HoodConstants.Limits.hoodHardStopMin);
        }));

        // PREPARE_SHOT: spin up, aim hood, keep feedStop engaged
        stateTransitionTrigger(ShotCycleState.PREPARE_SHOT).whenActive(new InstantCommand(() -> {
            // Transitioning to PREPARE_SHOT state
            this.robot.feedStop.disengage();
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
            //this.robot.shooterFeed.setFeedPower(ShooterFeedConstants.Targets.defaultFeedPower);
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

    public boolean doAutoHeading() {
        return this.currentState == ShotCycleState.PREPARE_SHOT ||
                this.currentState == ShotCycleState.READY_TO_SHOOT ||
                this.currentState == ShotCycleState.SHOOTING ||
                this.currentState == ShotCycleState.RECOVERY;
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

        table.add(0, 1850);

        // Big Triangle / Close Shots
        table.add(97, 1950);
        table.add(118, 2050);
        table.add(144, 2100);
        table.add(177, 2200);
        table.add(190, 2275);
        table.add(207, 2375);
        table.add(245, 2550);
        table.add(260, 2550);
        // Small Triangle / Far Shots
        table.add(325, 2860);
        table.add(338, 2900);
        table.add(347, 3060);
        table.add(355, 3000);
        table.add(380, 3100);
        table.add(392, 3140);


        table.add(600, 3200);
        table.createLUT();
        return table;
    }

    private InterpLUT setupHoodAngleInterpolator() {
        InterpLUT table = new InterpLUT();

        table.add(0, 275);

        // Big Triangle / Close Shots
        table.add(60, 110);
        table.add(81, 50);
        table.add(107, 20);
        table.add(140, 10);
        table.add(170, 10);
        table.add(208, 10);
        table.add(223, 10);

        // Small Triangle / Far Shots
        table.add(280, 10);
        table.add(295, 10);
        table.add(315, 10);
        table.add(330, 10);
        table.add(392, 10);

        table.add(600, 10);

        table.createLUT();
        return table;
    }
}