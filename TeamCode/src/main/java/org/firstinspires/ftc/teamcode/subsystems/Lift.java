package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Devices;
import org.firstinspires.ftc.teamcode.constants.LiftConstants;

public class Lift extends SubsystemBase {
    private final ServoEx liftServo;
    private final Telemetry telemetry;

    public enum LiftState {
        Engaged(LiftConstants.Positions.engagedDegrees),
        Disengaged(LiftConstants.Positions.disengagedDegrees),
        TestPosition(LiftConstants.Positions.testPositionDegrees);

        public final double positionDegrees;

        private LiftState(double degrees) {
            positionDegrees = degrees;
        }
    }

    private LiftState subsystemState = LiftState.Disengaged;
    private double positionDegrees = LiftConstants.Positions.testPositionDegrees;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        liftServo = new ServoEx(hardwareMap, Devices.Lift.liftServo,
                LiftConstants.DeviceConfig.liftServoAngleMin,
                LiftConstants.DeviceConfig.liftServoAngleMax);
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        switch (subsystemState) {
            case Engaged:
            case Disengaged:
                setPosition(subsystemState.positionDegrees);
                break;
            case TestPosition:
                break;
        }
        this.liftServo.set(this.positionDegrees);

        this.telemetry.addData("Lift: Position (Deg)", this.positionDegrees);
    }

    public void engage() {
        setPosition(LiftState.Engaged.positionDegrees);
        this.subsystemState = LiftState.Engaged;
    }

    public void disengage() {
        setPosition(LiftState.Disengaged.positionDegrees);
        this.subsystemState = LiftState.Disengaged;
    }

    public void setTestPosition() {
        setPosition(LiftConstants.Positions.testPositionDegrees);
        this.subsystemState = LiftState.TestPosition;
    }

    public void reset() {
        setPosition(LiftConstants.Positions.middleOfRange);
        this.subsystemState = LiftState.TestPosition;
    }

    public void moveBy(double degrees) {
        setPosition(this.positionDegrees + degrees);
    }

    private void setPosition(double degrees) {
        this.positionDegrees = MathUtils.clamp(degrees,
                LiftConstants.Limits.liftHardStopMin,
                LiftConstants.Limits.liftHardStopMax);
    }
}
