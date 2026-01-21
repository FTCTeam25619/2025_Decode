package org.firstinspires.ftc.teamcode.triggers;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.constants.IndicatorLightsConstants;
import org.firstinspires.ftc.teamcode.subsystems.IndicatorLights;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;

/**
 * Manages robot triggers and indicator light updates.
 *
 * Indicator lights serve multiple purposes:
 *
 * Lights 1-3 (Dual Purpose):
 * - INTAKE MODE: Show ball detection (GREEN/PURPLE/OFF for each sensor)
 * - SHOOTING MODE: Show readiness status (RED/GREEN for RPM, Ready, Heading)
 *
 * Light 4 (Independent):
 * - Auto-shoot mode: GREEN
 * - Preset shot mode: RED
 */
public class RobotTriggers {
    private final NovaPyraRobot robot;

    public RobotTriggers(NovaPyraRobot robot) {
        this.robot = robot;
    }

    /**
     * Setup triggers that aren't controller based.
     * Includes shot controller state transitions and mode indicator light.
     */
    public void setupNonControllerTriggers() {
        this.robot.shotController.setupStateTransitionTriggers();
        this.setupModeIndicatorTriggers();
    }

    /**
     * Update indicator lights 1-3 based on current robot state.
     * Called  from Explorer run() loop every cycle.
     * - Shooting mode overrides intake mode
     * - No repeated InstantCommand
     */
    public void updateIndicatorLights() {
        if (robot.shotController.isShooting()) {
            updateShootingLights();
        } else {
            updateIntakeLights();
        }
    }

    // ========================================================================
    // SHOOTING MODE: Show readiness indicators (Lights 1-3)
    // ========================================================================

    /**
     * Shooting mode light assignments:
     * - Light 1 (Top):   Shooter RPM status (RED = spinning up, GREEN = at target)
     * - Light 2 (Center): Overall ready to shoot (RED = not ready, GREEN = ready)
     * - Light 3 (Back):  Heading alignment (RED = turning, GREEN = aligned)
     */
    private void updateShootingLights() {
        // Light 1: Shooter RPM at target
        if (robot.shotController.isRPMatTarget()) {
            robot.indicatorLights.setLightColor(
                    IndicatorLights.Indicators.Light1,
                    IndicatorLightsConstants.IndicatorColors.GREEN
            );
        } else {
            robot.indicatorLights.setLightColor(
                    IndicatorLights.Indicators.Light1,
                    IndicatorLightsConstants.IndicatorColors.RED
            );
        }

        // Light 2: Overall ready to shoot (all conditions met)
        if (robot.shotController.readyToShoot()) {
            robot.indicatorLights.setLightColor(
                    IndicatorLights.Indicators.Light2,
                    IndicatorLightsConstants.IndicatorColors.GREEN
            );
        } else {
            robot.indicatorLights.setLightColor(
                    IndicatorLights.Indicators.Light2,
                    IndicatorLightsConstants.IndicatorColors.RED
            );
        }

        // Light 3: Robot heading aligned to target
        if (robot.vision.atHeading()) {
            robot.indicatorLights.setLightColor(
                    IndicatorLights.Indicators.Light3,
                    IndicatorLightsConstants.IndicatorColors.GREEN
            );
        } else {
            robot.indicatorLights.setLightColor(
                    IndicatorLights.Indicators.Light3,
                    IndicatorLightsConstants.IndicatorColors.RED
            );
        }
    }

    // ========================================================================
    // INTAKE MODE: Show ball detection by sensor location (Lights 1-3)
    // ========================================================================

    /**
     * Intake mode light assignments:
     * - Light 1 (Top):   Intake sensor (shows ball color or OFF)
     * - Light 2 (Center): Feed Low sensor (shows ball color or OFF)
     * - Light 3 (Back):  Feed High sensor (shows ball color or OFF)
     *
     * Colors:
     * - GREEN: Green ball detected
     * - PURPLE: Purple ball detected
     * - OFF: No ball detected
     */
    private void updateIntakeLights() {
        // Light 1: Intake sensor
        updateSensorLight(
                IndicatorLights.Indicators.Light1,
                Sensors.SensorLocation.Intake
        );

        // Light 2: Feed Low sensor
        updateSensorLight(
                IndicatorLights.Indicators.Light2,
                Sensors.SensorLocation.FeedLow
        );

        // Light 3: Feed High sensor
        updateSensorLight(
                IndicatorLights.Indicators.Light3,
                Sensors.SensorLocation.FeedHigh
        );
    }

    /**
     * Helper: Set light color based on what sensor sees.
     * Priority: GREEN > PURPLE > OFF
     */
    private void updateSensorLight(IndicatorLights.Indicators light,
                                   Sensors.SensorLocation sensor) {
        if (robot.sensors.seesGreenBall(sensor)) {
            robot.indicatorLights.setLightColor(
                    light,
                    IndicatorLightsConstants.IndicatorColors.GREEN
            );
        } else if (robot.sensors.seesPurpleBall(sensor)) {
            robot.indicatorLights.setLightColor(
                    light,
                    IndicatorLightsConstants.IndicatorColors.PURPLE
            );
        } else {
            robot.indicatorLights.setLightColor(
                    light,
                    IndicatorLightsConstants.IndicatorColors.OFF
            );
        }
    }

    // ========================================================================
    // MODE INDICATOR: Auto-shoot vs Preset mode (Light 4)
    // ========================================================================

    /**
     * Setup trigger-based updates for Light 4 (mode indicator).
     * This light is event-driven (only changes when driver switches modes)
     *
     * Light 4 assignments:
     * - GREEN: Auto-shoot mode (LUT-based targeting)
     * - RED: Preset shot mode (manual shot selection)
     */
    private void setupModeIndicatorTriggers() {
        // When entering auto-shoot mode → GREEN
        robot.shotController.presetShotMode().negate()
                .whenActive(new InstantCommand(() -> {
                    robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light4,
                            IndicatorLightsConstants.IndicatorColors.GREEN
                    );
                }));

        // When entering preset shot mode → RED
        robot.shotController.presetShotMode()
                .whenActive(new InstantCommand(() -> {
                    robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light4,
                            IndicatorLightsConstants.IndicatorColors.RED
                    );
                }));
    }
}