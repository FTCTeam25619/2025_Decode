package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.Devices;
import org.firstinspires.ftc.teamcode.constants.FeedStopConstants;

public class FeedStop extends SubsystemBase {
    private final ServoEx feedStopServo;
    private final Telemetry telemetry;

    public enum FeedStopState {
        Engaged(FeedStopConstants.Positions.engagedDegrees),
        Disengaged(FeedStopConstants.Positions.disengagedDegrees),
        TestPosition(FeedStopConstants.Positions.testPositionDegrees);

        public final double positionDegrees;

        private FeedStopState(double degrees) {
            positionDegrees = degrees;
        }
    }

    private FeedStopState subsystemState = FeedStopState.Disengaged;
    private double positionDegrees = FeedStopConstants.Positions.testPositionDegrees;

    public FeedStop(HardwareMap hardwareMap, Telemetry telemetry) {
        feedStopServo = new ServoEx(hardwareMap, Devices.FeedStop.feedStopServo,
                FeedStopConstants.DeviceConfig.feedStopServoAngleMin,
                FeedStopConstants.DeviceConfig.feedStopServoAngleMax);
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
        this.feedStopServo.set(this.positionDegrees);

        this.telemetry.addData("Feed Stop: Position (Deg)", this.positionDegrees);
    }

    public void engage() {
        setPosition(FeedStopState.Engaged.positionDegrees);
        this.subsystemState = FeedStopState.Engaged;
    }

    public void disengage() {
        setPosition(FeedStopState.Disengaged.positionDegrees);
        this.subsystemState = FeedStopState.Disengaged;
    }

    public void setTestPosition() {
        setPosition(FeedStopConstants.Positions.testPositionDegrees);
        this.subsystemState = FeedStopState.TestPosition;
    }

    public void moveBy(double degrees) {
        setPosition(this.positionDegrees + degrees);
    }

    private void setPosition(double degrees) {
        this.positionDegrees = MathUtils.clamp(degrees,
                FeedStopConstants.Limits.feedStopHardStopMin,
                FeedStopConstants.Limits.feedStopHardStopMax);
    }
}
