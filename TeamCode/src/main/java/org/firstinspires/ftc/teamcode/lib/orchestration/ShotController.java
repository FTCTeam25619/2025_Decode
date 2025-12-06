package org.firstinspires.ftc.teamcode.lib.orchestration;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.constants.HoodConstants;
import org.firstinspires.ftc.teamcode.constants.OdometryConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterFeedConstants;
import org.firstinspires.ftc.teamcode.lib.triggers.SupplierTrigger;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeed;

import java.util.function.Predicate;

public class ShotController {
    private SupplierTrigger _shotStaged;
    private SupplierTrigger _readyToShoot;

    private ShotCycleState currentState = ShotCycleState.IDLE;
    private ShotCycleState priorState = ShotCycleState.IDLE;

    private boolean stateTransition;

    private final NovaPyraRobot robot;
    private final Telemetry telemetry;

    private double targetFlywheelRPM = ShooterConstants.Targets.idleRPM;
    private double targetHoodAngle = HoodConstants.Limits.hoodHardStopMax;
    private double targetFeedPower = ShooterFeedConstants.Targets.defaultFeedPower;
    private boolean presetShotMode = true;

    private final InterpLUT shooterRPMInterpolator;
    private final InterpLUT hoodAngleInterpolator;
    private final InterpLUT feedPowerInterpolator;

    public ShotController(NovaPyraRobot robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;

        this.shooterRPMInterpolator = setupShooterRPMInterpolator();
        this.hoodAngleInterpolator = setupHoodAngleInterpolator();
        this.feedPowerInterpolator = setupFeedPowerInterpolator();
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
        telemetry.addData("ShotController: RPM", this.getTargetFlywheelRPM());
        telemetry.addData("ShotController: Angle", this.getTargetHoodAngle());
        telemetry.addData("ShotController: Feed Power", this.getTargetFeedPower());

        this.priorState = this.currentState;

        // State actions that need to happen each cycle
        switch (this.currentState) {
            case IDLE:
                if (this.staged()) {
                    this.currentState = ShotCycleState.STAGED;
                }
                break;
            case INTAKE:
                break;
            case STAGED:
                break;
            case PREPARE_SHOT:
                this.robot.shooter.setTargetRPM(this.getTargetFlywheelRPM());
                this.robot.hood.setAngle(this.getTargetHoodAngle());
                if (this.readyToShoot()) {
                    this.currentState = ShotCycleState.READY_TO_SHOOT;
                }
                break;
            case READY_TO_SHOOT:
                if (this.readyToShoot() && !presetShotMode) {
                    this.currentState = ShotCycleState.SHOOTING;
                }
                break;
            case SHOOTING:
                break;
        }
    }

    public void setupStateTransitionTriggers() {
        // TODO: Check/test the actions on these
        stateTransitionTrigger(ShotCycleState.IDLE).whenActive(new InstantCommand(() -> {
            // Transitioning back to IDLE state
            // NOTE: This will not trigger at start because initial state is IDLE
            this.robot.intake.setState(Intake.IntakeState.Stopped);
            this.robot.feedStop.disengage();
            this.robot.shooter.setTargetRPM(ShooterConstants.Targets.idleRPM);
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
            this.robot.hood.setAngle(HoodConstants.Limits.hoodHardStopMax);
        }));
        stateTransitionTrigger(ShotCycleState.INTAKE).whenActive(new InstantCommand(() -> {
            // Transitioning to INTAKE state
            this.robot.intake.setState(Intake.IntakeState.Intake);
            this.robot.feedStop.engage();
            // Same as prior state IDLE
            this.robot.shooter.setTargetRPM(ShooterConstants.Targets.idleRPM);
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
            this.robot.hood.setAngle(HoodConstants.Limits.hoodHardStopMax);
        }));
        stateTransitionTrigger(ShotCycleState.STAGED).whenActive(new InstantCommand(() -> {
            // Transitioning to STAGED state
            this.robot.intake.setState(Intake.IntakeState.Stopped);
            // Same as prior state INTAKE
            this.robot.feedStop.engage();
            this.robot.shooter.setTargetRPM(ShooterConstants.Targets.idleRPM);
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
            this.robot.hood.setAngle(HoodConstants.Limits.hoodHardStopMax);
        }));
        stateTransitionTrigger(ShotCycleState.PREPARE_SHOT).whenActive(new InstantCommand(() -> {
            // Transitioning to PREPARE_SHOT state
            this.robot.feedStop.engage();
            this.robot.shooter.setTargetRPM(this.getTargetFlywheelRPM());
            this.robot.hood.setAngle(this.getTargetHoodAngle());
            this.robot.intake.setState(Intake.IntakeState.Stopped);
            // Same as prior state STAGED
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
        }));
        stateTransitionTrigger(ShotCycleState.READY_TO_SHOOT).whenActive(new InstantCommand(() -> {
            // Transitioning to READY_TO_SHOOT state
            this.robot.feedStop.disengage();
            // Same as prior state PREPARE_SHOT
            this.robot.shooter.setTargetRPM(this.getTargetFlywheelRPM());
            this.robot.hood.setAngle(this.getTargetHoodAngle());
            this.robot.intake.setState(Intake.IntakeState.Intake);
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
        }));
        stateTransitionTrigger(ShotCycleState.SHOOTING).whenActive(new InstantCommand(() -> {
            // Transitioning to SHOOTING state
            // TODO: Extract this to a new InterpLUT and add to PresetShot values
            this.robot.shooterFeed.setFeedPower(this.getTargetFeedPower());
            this.robot.shooterFeed.setState(ShooterFeed.FeedState.Feeding);
            this.robot.intake.setState(Intake.IntakeState.Intake);
            // Same as prior state READY_TO_SHOOT
            this.robot.feedStop.disengage();
            this.robot.shooter.setTargetRPM(this.getTargetFlywheelRPM());
            this.robot.hood.setAngle(this.getTargetHoodAngle());
        }));
    }

    public void setTargetFlywheelRPM(double newTarget) {
        this.targetFlywheelRPM = newTarget;
    }

    public void setTargetHoodAngle(double newTarget) {
        this.targetHoodAngle = newTarget;
    }

    public void setTargetFeedPower(double newTarget) {
        this.targetFeedPower = newTarget;
    }

    public void autoTargetShot() {
        this.presetShotMode = false;
        double distance = getDistanceToTarget();
        setTargetFlywheelRPM(shooterRPMInterpolator.get(distance));
        setTargetHoodAngle(hoodAngleInterpolator.get(distance));
        setTargetFeedPower(feedPowerInterpolator.get(distance));
    }

    public void setupPresetShot(PresetShot shotSetting) {
        this.presetShotMode = true;
        setTargetFlywheelRPM(shotSetting.targetFlywheelRPM);
        setTargetHoodAngle(shotSetting.targetHoodAngle);
        setTargetFeedPower(shotSetting.targetFeedPower);
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

    public boolean staged() {
        return this.robot.sensors.seeingSomeBall();
    }

    public boolean readyToShoot() {
        return this.robot.shooter.atRPM() && this.robot.hood.atTargetAngle();
    }

    public boolean transitionedTo(ShotCycleState newState) {
        return this.stateTransition && this.currentState == newState;
    }

    public SupplierTrigger shotStagedTrigger() {
        if (this._shotStaged == null) {
            this._shotStaged = new SupplierTrigger(this::staged);
        }
        return this._shotStaged;
    }

    public SupplierTrigger readyToShootTrigger() {
        if (this._readyToShoot == null) {
            this._readyToShoot = new SupplierTrigger(this::readyToShoot);
        }
        return this._readyToShoot;
    }

    public StateTransitionTrigger stateTransitionTrigger(ShotCycleState toState) {
        return new StateTransitionTrigger(this::transitionedTo, toState);
    }

    public static class StateTransitionTrigger extends Trigger {
        ShotCycleState checkState;
        Predicate<ShotCycleState> getFn;
        public StateTransitionTrigger(Predicate<ShotCycleState> shotCycleStateSupplier, ShotCycleState state) {
            this.checkState = state;
            this.getFn = shotCycleStateSupplier;
        }

        @Override
        public boolean get() { return this.getFn.test(checkState); }
    }

    private double getDistanceToTarget() {
        // TODO: Actually implement logic to read distance from LL/Odometry
        return OdometryConstants.TargetDistances.nearZoneEndShot;
    }

    private InterpLUT setupShooterRPMInterpolator() {
        InterpLUT table = new InterpLUT();
        table.add(OdometryConstants.TargetDistances.atGoalShot, ShooterConstants.Targets.atGoalShot);
        table.add(OdometryConstants.TargetDistances.nearZoneMidShot, ShooterConstants.Targets.nearZoneMidShot);
        table.add(OdometryConstants.TargetDistances.nearZoneEndShot, ShooterConstants.Targets.nearZoneEndShot);
        table.add(OdometryConstants.TargetDistances.farWallShot, ShooterConstants.Targets.farWallShot);
        table.createLUT();
        return table;
    }

    private InterpLUT setupHoodAngleInterpolator() {
        InterpLUT table = new InterpLUT();
        table.add(OdometryConstants.TargetDistances.atGoalShot, HoodConstants.Targets.atGoalShot);
        table.add(OdometryConstants.TargetDistances.nearZoneMidShot, HoodConstants.Targets.nearZoneMidShot);
        table.add(OdometryConstants.TargetDistances.nearZoneEndShot, HoodConstants.Targets.nearZoneEndShot);
        table.add(OdometryConstants.TargetDistances.farWallShot, HoodConstants.Targets.farWallShot);
        table.createLUT();
        return table;
    }

    private InterpLUT setupFeedPowerInterpolator() {
        InterpLUT table = new InterpLUT();
        table.add(OdometryConstants.TargetDistances.atGoalShot, ShooterFeedConstants.Targets.atGoalShot);
        table.add(OdometryConstants.TargetDistances.nearZoneMidShot, ShooterFeedConstants.Targets.nearZoneMidShot);
        table.add(OdometryConstants.TargetDistances.nearZoneEndShot, ShooterFeedConstants.Targets.nearZoneEndShot);
        table.add(OdometryConstants.TargetDistances.farWallShot, ShooterFeedConstants.Targets.farWallShot);
        table.createLUT();
        return table;
    }
}
