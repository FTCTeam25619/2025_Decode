package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.Devices;
import org.firstinspires.ftc.teamcode.lib.servos.ContinuousServo;

public class ShooterFeed extends SubsystemBase {
    private final ContinuousServo feedServoLeft;
    private final ContinuousServo feedServoRight;

    private double feedPower = 0.0;

    private final Telemetry telemetry;

    public enum FeedState {
        Stopped,
        Feeding,
        Dejam;
    }

    public FeedState subsystemState = FeedState.Stopped;

    public ShooterFeed(HardwareMap hardwareMap, Telemetry telemetry) {
        this.feedServoLeft = new ContinuousServo(hardwareMap, Devices.ShooterFeed.feedCRServoLeft);
        this.feedServoRight = new ContinuousServo(hardwareMap, Devices.ShooterFeed.feedCRServoRight);
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        switch(subsystemState) {
            case Stopped:
                feedServoLeft.stop();
                feedServoRight.stop();
                break;
            case Dejam:
                feedServoLeft.reverse(this.feedPower);
                feedServoRight.reverse(this.feedPower);
                break;
            case Feeding:
                feedServoLeft.forward(this.feedPower);
                feedServoRight.forward(this.feedPower);
                break;
        }

        this.telemetry.addData("ShooterFeed: State", subsystemState.toString());
        this.telemetry.addData("ShooterFeed: Power", this.feedPower);
    }

    public void setFeedPower(double power) {
        this.feedPower = MathUtils.clamp(power, 0.0, 1.0);
    }

    public void changeFeedPower(double powerChange) {
        setFeedPower(this.feedPower + powerChange);
    }

    public void setState(FeedState newState) {
        this.subsystemState = newState;
    }
}
