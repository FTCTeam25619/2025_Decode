package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Devices;
import org.firstinspires.ftc.teamcode.constants.HoodConstants;

public class Hood extends SubsystemBase {
    private final ServoEx hoodServo;
    private final Telemetry telemetry;

    private double currentAngle = (HoodConstants.Limits.hoodHardStopMin +
            HoodConstants.Limits.hoodHardStopMax) / 2.0;

    public Hood(HardwareMap hardwareMap, Telemetry telemetry) {
        hoodServo = new ServoEx(hardwareMap, Devices.Hood.hoodServo,
                HoodConstants.DeviceConfig.hoodServoAngleMin,
                HoodConstants.DeviceConfig.hoodServoAngleMax);
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        hoodServo.set(currentAngle);

        telemetry.addData("Hood: Angle", currentAngle);
    }

    public void setAngle(double degrees) {
        currentAngle = MathUtils.clamp(degrees,
                HoodConstants.Limits.hoodHardStopMin,
                HoodConstants.Limits.hoodHardStopMax);
    }
}
