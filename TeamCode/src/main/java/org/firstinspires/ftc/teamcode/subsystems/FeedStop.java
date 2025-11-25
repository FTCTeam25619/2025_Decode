package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.Devices;
import org.firstinspires.ftc.teamcode.constants.FeedStopConstants;

public class FeedStop extends SubsystemBase {
    private final ServoEx feedStopServo;
    private final Telemetry telemetry;

    private double positionDegrees = FeedStopConstants.disengagedDegrees;

    public enum FeedStopState {
        Engaged(FeedStopConstants.engagedDegrees),
        Disengaged(FeedStopConstants.disengagedDegrees),
        TestPosition(FeedStopConstants.testPositionDegrees);

        public double positionDegrees;

        private FeedStopState(double degrees) {
            positionDegrees = degrees;
        }
    }

    private FeedStopState subsystemState = FeedStopState.Disengaged;

    public FeedStop(HardwareMap hardwareMap, Telemetry telemetry) {
        feedStopServo = new ServoEx(hardwareMap, Devices.FeedStop.feedStopServo, 300.0, AngleUnit.DEGREES);
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        switch (subsystemState) {
            case Engaged:
            case Disengaged:
                this.positionDegrees = subsystemState.positionDegrees;
                break;
            case TestPosition:
                this.positionDegrees = FeedStopConstants.testPositionDegrees;
                break;
        }
//        this.feedStopServo.set(this.positionDegrees);

        this.telemetry.addData("Feed Stop: Position (Deg)", this.positionDegrees);
    }

    public void engage() {
        this.positionDegrees = FeedStopState.Engaged.positionDegrees;
        this.subsystemState = FeedStopState.Engaged;
    }

    public void disengage() {
        this.positionDegrees = FeedStopState.Disengaged.positionDegrees;
        this.subsystemState = FeedStopState.Disengaged;
    }

    public void setTestPosition(double degrees) {
        this.positionDegrees = FeedStopConstants.testPositionDegrees;
        this.subsystemState = FeedStopState.TestPosition;
    }
}
