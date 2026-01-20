package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Devices;
import org.firstinspires.ftc.teamcode.constants.PowerConstants;

public class Power extends SubsystemBase {
    private final AnalogInput currentSensor;
    private final double ampsPerVolt;

    private final Telemetry telemetry;

    public Power(HardwareMap hardwareMap, Telemetry telemetry) {
        this.currentSensor = hardwareMap.get(AnalogInput.class, Devices.Power.currentSensor);
        this.ampsPerVolt = PowerConstants.maxAmperage / this.currentSensor.getMaxVoltage();

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        this.telemetry.addData("Current Draw (A)", this.getCurrentDraw());
    }

    public double getCurrentDraw() {
        return this.currentSensor.getVoltage() * this.ampsPerVolt;
    }
}
