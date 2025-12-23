package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Devices;
import org.firstinspires.ftc.teamcode.constants.HoodConstants;

public class Hood extends SubsystemBase {
    private final ServoEx hoodServo;
    private final AnalogInput hoodEncoder;
    private final Telemetry telemetry;

    private double currentAngle = (HoodConstants.Limits.hoodHardStopMin +
            HoodConstants.Limits.hoodHardStopMax) / 2.0;

    public Hood(HardwareMap hardwareMap, Telemetry telemetry) {
        hoodServo = new ServoEx(hardwareMap, Devices.Hood.hoodServo,
                HoodConstants.DeviceConfig.hoodServoAngleMin,
                HoodConstants.DeviceConfig.hoodServoAngleMax);
        hoodEncoder = hardwareMap.get(AnalogInput.class, Devices.Hood.hoodEncoder);
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        hoodServo.set(currentAngle);

        telemetry.addData("Hood: Angle", currentAngle);
        telemetry.addData("Hood: Enc V", readEncoderVoltage());
        telemetry.addData("Hood: Enc Deg", encoderAngle());
    }

    public void setAngle(double degrees) {
        currentAngle = MathUtils.clamp(degrees,
                HoodConstants.Limits.hoodHardStopMin,
                HoodConstants.Limits.hoodHardStopMax);
    }

    public boolean atTargetAngle() {
        // FIXME: Replace this with actual thresholding of readings from servo feedback when available
        return true;
    }

    public double readEncoderVoltage() {
        return hoodEncoder.getVoltage();
    }

    public double encoderAngle() {
        return readEncoderVoltage() * (-123.609) + 383.27;
    }

    public void moveBy(double degrees) {
        setAngle(currentAngle + degrees);
    }
}
