package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.lights.LightsManager;
import com.bylazar.lights.PanelsLights;
import com.bylazar.lights.RGBIndicator;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.SensorRevColorV3;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.GamePieceColorConstants;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.Devices;
import org.firstinspires.ftc.teamcode.constants.IndicatorLightsConstants;
import org.firstinspires.ftc.teamcode.lib.GamePieceColor;
import org.firstinspires.ftc.teamcode.lib.triggers.SupplierTrigger;

public class Sensors extends SubsystemBase {

    private final Telemetry telemetry;

    // For Panels telemetry display of sensor values (only when DEBUG_SENSORS true)
    private final LightsManager lightTelemetryManager;
    private final TelemetryManager panelsTelemetry;

    public enum SensorLocation {
        Intake,
        FeedLow,
        FeedHigh;

        public SensorRevColorV3 sensor;
        public RGBIndicator colorIndicator;
        public RGBIndicator ballIndicator;
        public float hue = 0.0f;
        public double distance = GamePieceColorConstants.Colors.MATCHING_DISTANCE_CM + 100.0;

        SensorLocation() {}

        public void setSensor(SensorRevColorV3 sensor) {
            this.sensor = sensor;
        }

        public void setTelemetry(RGBIndicator colorIndicator, RGBIndicator ballIndicator) {
            this.colorIndicator = colorIndicator;
            this.ballIndicator = ballIndicator;
        }

        public void update() {
            if (this.sensor != null) {
                hue = getHue(this.sensor);
                distance = getDistance(this.sensor);
            }
        }

        public boolean hueIsGreen() {
            return GamePieceColor.GREEN.matches(hue);
        }

        public boolean hueIsPurple() {
            return GamePieceColor.PURPLE.matches(hue);
        }

        public boolean seeingBall() {
            return distance <= GamePieceColorConstants.Colors.MATCHING_DISTANCE_CM;
        }

        public void updateTelemetry() {
            if (Constants.DebugModes.Telemetry.DEBUG_SENSORS) {
                if (colorIndicator != null) {
                    if (hueIsGreen()) {
                        colorIndicator.update(IndicatorLightsConstants.IndicatorColors.GREEN.pwmSetting);
                    } else if (hueIsPurple()) {
                        colorIndicator.update(IndicatorLightsConstants.IndicatorColors.PURPLE.pwmSetting);
                    } else {
                        colorIndicator.update(IndicatorLightsConstants.IndicatorColors.OFF.pwmSetting);
                    }
                }
                if (ballIndicator != null) {
                    if (seeingBall()) {
                        ballIndicator.update(RGBIndicator.Companion.getGREEN());
                    } else {
                        ballIndicator.update(RGBIndicator.Companion.getRED());
                    }
                }
            }
        }

        private float getHue(SensorRevColorV3 sensor) {
            int red = sensor.red();
            int green = sensor.green();
            int blue = sensor.blue();
            float[] hsv = new float[3];
            sensor.RGBtoHSV(red, green, blue, hsv);
            return hsv[0];
        }

        private double getDistance(SensorRevColorV3 sensor) { return sensor.distance(); }
    }

    public Sensors(HardwareMap hardwareMap, Telemetry telemetry) {
        SensorLocation.Intake.setSensor(new SensorRevColorV3(
                hardwareMap, Devices.Sensors.intakeColorSensor, DistanceUnit.CM));
        SensorLocation.FeedLow.setSensor(new SensorRevColorV3(
                hardwareMap, Devices.Sensors.feedColorSensorLow, DistanceUnit.CM));
        SensorLocation.FeedHigh.setSensor(new SensorRevColorV3(
                hardwareMap, Devices.Sensors.feedColorSensorHigh, DistanceUnit.CM));

        this.telemetry = telemetry;

        if (Constants.DebugModes.Telemetry.DEBUG_SENSORS) {
            // Setup debug telemetry
            lightTelemetryManager = PanelsLights.INSTANCE.getLights();
            panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
            SensorLocation.Intake.setTelemetry(
                    new RGBIndicator("Intake Color"),
                    new RGBIndicator("Intake Ball"));
            SensorLocation.FeedLow.setTelemetry(
                    new RGBIndicator("FeedLow Color"),
                    new RGBIndicator("FeedLow Ball"));
            SensorLocation.FeedHigh.setTelemetry(
                    new RGBIndicator("FeedHigh Color"),
                    new RGBIndicator("FeedHigh Ball"));
            lightTelemetryManager.initLights(
                    SensorLocation.Intake.colorIndicator,
                    SensorLocation.FeedLow.colorIndicator,
                    SensorLocation.FeedHigh.colorIndicator,
                    SensorLocation.Intake.ballIndicator,
                    SensorLocation.FeedLow.ballIndicator,
                    SensorLocation.FeedHigh.ballIndicator
            );
        } else {
            lightTelemetryManager = null;
            panelsTelemetry = null;
        }
    }

    @Override
    public void periodic() {
        for (SensorLocation location : SensorLocation.values()) {
            location.update();
            if (Constants.DebugModes.Telemetry.DEBUG_SENSORS) { location.updateTelemetry(); }
        }

        displayTelemetry();
        if (Constants.DebugModes.Telemetry.DEBUG_SENSORS) { displayDebugTelemetry(); }
    }

    public boolean seeingSomeBall(SensorLocation location) {
        return location.seeingBall() && (location.hueIsGreen() || location.hueIsPurple());
    }

    public boolean seeingSomeBall() {
        return seeingSomeBall(SensorLocation.Intake) ||
                seeingSomeBall(SensorLocation.FeedLow) ||
                seeingSomeBall(SensorLocation.FeedHigh);
    }

    public boolean seesGreenBall(SensorLocation location) {
        return location.seeingBall() && location.hueIsGreen();
    }

    public boolean seesPurpleBall(SensorLocation location) {
        return location.seeingBall() && location.hueIsPurple();
    }

    public SupplierTrigger intakeSeesGreen =
            new SupplierTrigger(() -> seesGreenBall(SensorLocation.Intake));
    public SupplierTrigger intakeSeesPurple =
            new SupplierTrigger(() -> seesPurpleBall(SensorLocation.Intake));
    public SupplierTrigger feedLowSeesGreen =
            new SupplierTrigger(() -> seesGreenBall(SensorLocation.FeedLow));
    public SupplierTrigger feedLowSeesPurple =
            new SupplierTrigger(() -> seesPurpleBall(SensorLocation.FeedLow));
    public SupplierTrigger feedHighSeesGreen =
            new SupplierTrigger(() -> seesGreenBall(SensorLocation.FeedHigh));
    public SupplierTrigger feedHighSeesPurple =
            new SupplierTrigger(() -> seesPurpleBall(SensorLocation.FeedHigh));

    private void displayTelemetry() {
        this.telemetry.addData("Sensors Intake Green Ball?", seesGreenBall(SensorLocation.Intake));
        this.telemetry.addData("Sensors Feed Low Green Ball?", seesGreenBall(SensorLocation.FeedLow));
        this.telemetry.addData("Sensors Feed High Green Ball?", seesGreenBall(SensorLocation.FeedHigh));
        this.telemetry.addData("Sensors Intake Purple Ball?", seesPurpleBall(SensorLocation.Intake));
        this.telemetry.addData("Sensors Feed Low Purple Ball?", seesPurpleBall(SensorLocation.FeedLow));
        this.telemetry.addData("Sensors Feed High Purple Ball?", seesPurpleBall(SensorLocation.FeedHigh));
    }

    private void displayDebugTelemetry() {
        lightTelemetryManager.update();
        lightTelemetryManager.addToTelemetry(telemetry);
        lightTelemetryManager.getLightsState().forEach((sensor) -> {
            panelsTelemetry.debug(
                    sensor.getId() + " " + sensor.getType() + " " + sensor.getValue());
        });
        panelsTelemetry.update();

        this.telemetry.addData("Sensors Intake Dist (cm)", SensorLocation.Intake.distance);
        this.telemetry.addData("Sensors Feed Low Dist (cm)", SensorLocation.FeedLow.distance);
        this.telemetry.addData("Sensors Feed High Dist (cm)", SensorLocation.FeedHigh.distance);
        this.telemetry.addData("Sensors Intake Ball?", SensorLocation.Intake.seeingBall());
        this.telemetry.addData("Sensors Feed Low Ball?", SensorLocation.FeedLow.seeingBall());
        this.telemetry.addData("Sensors Feed High Ball?", SensorLocation.FeedHigh.seeingBall());

        this.telemetry.addData("Sensors Intake Hue", SensorLocation.Intake.hue);
        this.telemetry.addData("Sensors Feed Low Hue", SensorLocation.FeedLow.hue);
        this.telemetry.addData("Sensors Feed High Hue", SensorLocation.FeedHigh.hue);
        this.telemetry.addData("Sensors Intake Green", SensorLocation.Intake.hueIsGreen());
        this.telemetry.addData("Sensors Feed Low Green", SensorLocation.FeedLow.hueIsGreen());
        this.telemetry.addData("Sensors Feed High Green", SensorLocation.FeedHigh.hueIsGreen());
        this.telemetry.addData("Sensors Intake Purple", SensorLocation.Intake.hueIsPurple());
        this.telemetry.addData("Sensors Feed Low Purple", SensorLocation.FeedLow.hueIsPurple());
        this.telemetry.addData("Sensors Feed High Purple", SensorLocation.FeedHigh.hueIsPurple());
    }
}
