package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Devices;
import org.firstinspires.ftc.teamcode.constants.LiftConstants;

public class Lift extends SubsystemBase {
    private final ServoEx liftServoLeft;
    private final ServoEx liftServoRight;
    private final Telemetry telemetry;

    public enum LiftState {
        Engaged(
                LiftConstants.Positions.LeftServo.engagedDegrees,
                LiftConstants.Positions.RightServo.engagedDegrees
        ),
        Disengaged(
                LiftConstants.Positions.LeftServo.disengagedDegrees,
                LiftConstants.Positions.RightServo.disengagedDegrees
        ),
        TestPosition(
                LiftConstants.Positions.LeftServo.testPositionDegrees,
                LiftConstants.Positions.RightServo.testPositionDegrees
        );

        public final double leftPositionDegrees;
        public final double rightPositionDegrees;

        private LiftState(double leftDegrees, double rightDegrees) {
            leftPositionDegrees = leftDegrees;
            rightPositionDegrees = rightDegrees;
        }
    }

    private LiftState subsystemState = LiftState.Disengaged;
    private double leftPositionDegrees = LiftConstants.Positions.LeftServo.middleOfRange;
    private double rightPositionDegrees = LiftConstants.Positions.RightServo.middleOfRange;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        liftServoLeft = new ServoEx(hardwareMap, Devices.Lift.liftServoLeft,
                LiftConstants.DeviceConfig.liftServoAngleMin,
                LiftConstants.DeviceConfig.liftServoAngleMax);
        liftServoRight = new ServoEx(hardwareMap, Devices.Lift.liftServoRight,
                LiftConstants.DeviceConfig.liftServoAngleMin,
                LiftConstants.DeviceConfig.liftServoAngleMax);


        liftServoLeft.setInverted(false);
        liftServoRight.setInverted(false);

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        switch (subsystemState) {
            case Engaged:
            case Disengaged:
                setPosition(
                        subsystemState.leftPositionDegrees,
                        subsystemState.rightPositionDegrees
                );
                break;
            case TestPosition:
                break;
        }
        this.liftServoLeft.set(this.leftPositionDegrees);
        this.liftServoRight.set(this.rightPositionDegrees);

        //this.telemetry.addData("Lift: Left Pos (Deg)", this.leftPositionDegrees);
        //this.telemetry.addData("Lift: Right Pos (Deg)", this.rightPositionDegrees);
    }

    public void engage() {
        setPosition(
                LiftState.Engaged.leftPositionDegrees,
                LiftState.Engaged.rightPositionDegrees
        );
        this.subsystemState = LiftState.Engaged;
    }

    public void disengage() {
        setPosition(
                LiftState.Disengaged.leftPositionDegrees,
                LiftState.Disengaged.rightPositionDegrees
        );
        this.subsystemState = LiftState.Disengaged;
    }

    public void setTestPosition() {
        setPosition(
                LiftConstants.Positions.LeftServo.testPositionDegrees,
                LiftConstants.Positions.RightServo.testPositionDegrees
        );
        this.subsystemState = LiftState.TestPosition;
    }

    public void reset() {
        setPosition(
                LiftConstants.Positions.LeftServo.middleOfRange,
                LiftConstants.Positions.RightServo.middleOfRange
        );
        this.subsystemState = LiftState.TestPosition;
    }

    public void moveBy(double degrees) {
        setPosition(
                this.leftPositionDegrees + degrees,
                this.rightPositionDegrees + degrees
        );
    }

    private void setPosition(double leftDegrees, double rightDegrees) {
        this.leftPositionDegrees = MathUtils.clamp(leftDegrees,
                LiftConstants.Limits.LeftServo.hardStopMin,
                LiftConstants.Limits.LeftServo.hardStopMax);
        this.rightPositionDegrees = MathUtils.clamp(rightDegrees,
                LiftConstants.Limits.RightServo.hardStopMin,
                LiftConstants.Limits.RightServo.hardStopMax);
    }
}
