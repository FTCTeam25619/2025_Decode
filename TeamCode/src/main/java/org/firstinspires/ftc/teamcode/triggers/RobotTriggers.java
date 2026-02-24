package org.firstinspires.ftc.teamcode.triggers;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.constants.IndicatorLightsConstants;
import org.firstinspires.ftc.teamcode.subsystems.IndicatorLights;

public class RobotTriggers {
    private final NovaPyraRobot robot;

    public RobotTriggers(NovaPyraRobot robot) {
        this.robot = robot;
    }

    public void setupNonControllerTriggers() {
        setupNonShootingBallPatternDisplayTriggers();
        setupShootingReadinessDisplayTriggers();
        setupIntakeStallDisplayTriggers();
        this.robot.shotController.setupStateTransitionTriggers();
    }
    public void setupIntakeStallDisplayTriggers() {
        //trigger that activates when intake is stalled
        Trigger intakeStalled = new Trigger(() -> this.robot.intake.isStalled());

        // Set all lights to BLUE when intake stalls (indicating 3 balls)
        intakeStalled.whenActive(new InstantCommand(() -> {
            this.robot.indicatorLights.setLightColor(
                    IndicatorLights.Indicators.Light1,
                    IndicatorLightsConstants.IndicatorColors.WHITE);
            this.robot.indicatorLights.setLightColor(
                    IndicatorLights.Indicators.Light2,
                    IndicatorLightsConstants.IndicatorColors.WHITE);
            this.robot.indicatorLights.setLightColor(
                    IndicatorLights.Indicators.Light3,
                    IndicatorLightsConstants.IndicatorColors.WHITE);
        }));
    }

    public void setupNonShootingBallPatternDisplayTriggers() {
        //  Light 1 OFF
        robot.shotController.isShootingTrigger().negate()
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light1,
                            IndicatorLightsConstants.IndicatorColors.OFF);
                }));

        //  Light 2 OFF
        robot.shotController.isShootingTrigger().negate()
               .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light2,
                            IndicatorLightsConstants.IndicatorColors.OFF);
                }));


        //  Light 3 OFF
        robot.shotController.isShootingTrigger().negate()
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light3,
                            IndicatorLightsConstants.IndicatorColors.OFF);
                }));
    }

    public void setupShootingReadinessDisplayTriggers() {
        // While in a shooting mode, if shooter RPM is not at target, Light 1 RED
        this.robot.shotController.isShootingTrigger()
                .and(this.robot.shotController.shooterAtRPM().negate())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light1,
                            IndicatorLightsConstants.IndicatorColors.RED);
                }));
        // While in a shooting mode, if shooter RPM is at target, Light 1 GREEN
        this.robot.shotController.isShootingTrigger()
                .and(this.robot.shotController.shooterAtRPM())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light1,
                            IndicatorLightsConstants.IndicatorColors.GREEN);
                }));
        // While in a shooting mode, if any ready to shoot condition not met, Light 2 RED
        this.robot.shotController.isShootingTrigger()
                .and(this.robot.shotController.readyToShootTrigger().negate())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light2,
                            IndicatorLightsConstants.IndicatorColors.RED);
                }));
        // While in a shooting mode, if all ready to shoot conditions met, Light 2 GREEN
        this.robot.shotController.isShootingTrigger()
                .and(this.robot.shotController.readyToShootTrigger())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light2,
                            IndicatorLightsConstants.IndicatorColors.GREEN);
                }));
        // While in a shooting mode, if robot heading is not at target, Light 3 RED
        this.robot.shotController.isShootingTrigger()
                .and(this.robot.shotController.robotAtHeading().negate())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light3,
                            IndicatorLightsConstants.IndicatorColors.RED);
                }));
        // While in a shooting mode, if robot heading is at target, Light 3 GREEN
        this.robot.shotController.isShootingTrigger()
                .and(this.robot.shotController.robotAtHeading())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light3,
                            IndicatorLightsConstants.IndicatorColors.GREEN);
                }));
    }
}
