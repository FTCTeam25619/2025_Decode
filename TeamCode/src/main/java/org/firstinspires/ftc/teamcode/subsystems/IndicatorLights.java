package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.lights.LightsManager;
import com.bylazar.lights.PanelsLights;
import com.bylazar.lights.RGBIndicator;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.Devices;
import org.firstinspires.ftc.teamcode.constants.IndicatorLightsConstants;

public class IndicatorLights extends SubsystemBase {
    private final ServoEx indicatorLight1;
    private final ServoEx indicatorLight2;
    private final ServoEx indicatorLight3;
    private final ServoEx indicatorLight4;
    private final Telemetry telemetry;

    private final LightsManager lightTelemetryManager;
    private final TelemetryManager panelsTelemetry;
    private final RGBIndicator light1Telemetry;
    private final RGBIndicator light2Telemetry;
    private final RGBIndicator light3Telemetry;
    private final RGBIndicator light4Telemetry;
    public enum Indicators {
        Light1,
        Light2,
        Light3,
        Light4;
    }

    public IndicatorLights(HardwareMap hardwareMap, Telemetry telemetry) {
        indicatorLight1 = new ServoEx(hardwareMap, Devices.Indicators.indicatorLight1);
        indicatorLight2 = new ServoEx(hardwareMap, Devices.Indicators.indicatorLight2);
        indicatorLight3 = new ServoEx(hardwareMap, Devices.Indicators.indicatorLight3);
        indicatorLight4 = new ServoEx(hardwareMap, Devices.Indicators.indicatorLight4);

        this.telemetry = telemetry;
        if (Constants.DebugModes.Telemetry.DEBUG_LIGHTS) {
            lightTelemetryManager = PanelsLights.INSTANCE.getLights();
            panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
            light1Telemetry = new RGBIndicator("Light 1");
            light2Telemetry = new RGBIndicator("Light 2");
            light3Telemetry = new RGBIndicator("Light 3");
            light4Telemetry = new RGBIndicator("Light 4");
            lightTelemetryManager.initLights(
                    light1Telemetry,
                    light2Telemetry,
                    light3Telemetry,
                    light4Telemetry
            );
        } else {
            lightTelemetryManager = null;
            panelsTelemetry = null;
            light1Telemetry = null;
            light2Telemetry = null;
            light3Telemetry = null;
            light4Telemetry = null;
        }
    }

    @Override
    public void periodic() {
        if (Constants.DebugModes.Telemetry.DEBUG_LIGHTS) {
            lightTelemetryManager.update();
            lightTelemetryManager.addToTelemetry(telemetry);
            lightTelemetryManager.getLightsState().forEach((light) -> {
                panelsTelemetry.debug(light.getId() + " " + light.getType() + " " + light.getValue());
            });
            panelsTelemetry.update();
        }
    }

    public void setLightColor(Indicators selectedLight, IndicatorLightsConstants.IndicatorColors color) {
        if (selectedLight == null) { return; }
        if (color == null) { color = IndicatorLightsConstants.IndicatorColors.OFF; }
        switch (selectedLight) {
            case Light1:
                setLightColor(indicatorLight1, color.pwmSetting);
                addLightTelemetry("Light 1", color, light1Telemetry);
                break;
            case Light2:
                setLightColor(indicatorLight2, color.pwmSetting);
                addLightTelemetry("Light 2", color, light2Telemetry);
                break;
            case Light3:
                setLightColor(indicatorLight3, color.pwmSetting);
                addLightTelemetry("Light 3", color, light3Telemetry);
                break;
            case Light4:
                setLightColor(indicatorLight4, color.pwmSetting);
                addLightTelemetry("Light 4", color, light4Telemetry);
                break;
        }
    }

    private void setLightColor(ServoEx light, double pwmSetting) {
        light.set(pwmSetting);
    }

    private void addLightTelemetry(String lightName, IndicatorLightsConstants.IndicatorColors color, RGBIndicator lightTelemetry) {
        String telemetryValue = color.name() + " (" + color.pwmSetting + ")";
        telemetry.addData(lightName, telemetryValue);
        if (Constants.DebugModes.Telemetry.DEBUG_LIGHTS && lightTelemetry != null) {
            lightTelemetry.update(color.pwmSetting);
        }
    }
}

/**
 * Flow Diagram for Light 4 (AutoAim / Preset Indicator)
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │ 1. DRIVER ACTION (CompetitionTriggers.java)                             │
 * └─────────────────────────────────────────────────────────────────────────┘
 *                                │
 *                                ▼
 *         ┌──────────────────────────────────────┐
 *         │ Back + A pressed                     │
 *         │ OR                                   │
 *         │ DPad Left pressed                    │
 *         └──────────────────────────────────────┘
 *                                │
 *                                ▼
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │ 2. SHOTCONTROLLER STATE CHANGE (ShotController.java)                    │
 * └─────────────────────────────────────────────────────────────────────────┘
 *                                │
 *          ┌─────────────────────┴─────────────────────┐
 *          ▼                                           ▼
 * ┌─────────────────────┐                   ┌─────────────────────┐
 * │ setupPresetShot()   │                   │ autoTargetShot()    │
 * │ presetShotMode=TRUE │                   │ presetShotMode=FALSE│
 * └─────────────────────┘                   └─────────────────────┘
 *          │                                           │
 *          └─────────────────────┬─────────────────────┘
 *                                ▼
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │ 3. TRIGGER FIRES (RobotTriggers.java)                                   │
 * │    Triggers check presetShotMode every cycle via SupplierTrigger        │
 * └─────────────────────────────────────────────────────────────────────────┘
 *                                │
 *          ┌─────────────────────┴─────────────────────┐
 *          ▼                                           ▼
 * ┌─────────────────────┐                   ┌─────────────────────┐
 * │ presetShotMode()    │                   │ presetShotMode()    │
 * │ returns TRUE        │                   │ .negate()           │
 * │ → whenActive fires  │                   │ returns TRUE        │
 * │                     │                   │ → whenActive fires  │
 * └─────────────────────┘                   └─────────────────────┘
 *          │                                           │
 *          ▼                                           ▼
 * ┌─────────────────────┐                   ┌─────────────────────┐
 * │ Set Light4 RED      │                   │ Set Light4 GREEN    │
 * └─────────────────────┘                   └─────────────────────┘
 *                                │
 *                                ▼
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │ 4. LIGHT UPDATES (IndicatorLights.java)                                 │
 * └─────────────────────────────────────────────────────────────────────────┘
 */

