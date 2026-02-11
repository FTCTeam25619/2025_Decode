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
                    IndicatorLightsConstants.IndicatorColors.BLUE);
            this.robot.indicatorLights.setLightColor(
                    IndicatorLights.Indicators.Light2,
                    IndicatorLightsConstants.IndicatorColors.BLUE);
            this.robot.indicatorLights.setLightColor(
                    IndicatorLights.Indicators.Light3,
                    IndicatorLightsConstants.IndicatorColors.BLUE);
        }));
    }

    public void setupNonShootingBallPatternDisplayTriggers() {
        // Light 1 reflects color of ball seen by intake sensor, if any
        // If intake sensor sees green, Light 1 GREEN
        robot.shotController.isShootingTrigger().negate()
                .and(this.robot.sensors.intakeSeesGreen)
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light1,
                            IndicatorLightsConstants.IndicatorColors.GREEN);
                }));
        // If intake sensor sees purple, Light 1 PURPLE
        robot.shotController.isShootingTrigger().negate()
                .and(this.robot.sensors.intakeSeesPurple)
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light1,
                            IndicatorLightsConstants.IndicatorColors.PURPLE);
                }));
        // If intake sensor sees neither, Light 1 OFF
        robot.shotController.isShootingTrigger().negate()
                .and(this.robot.sensors.intakeSeesGreen.negate())
                .and(this.robot.sensors.intakeSeesPurple.negate())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light1,
                            IndicatorLightsConstants.IndicatorColors.OFF);
                }));

        // Light 2 reflects color of ball seen by feed low sensor, if any
        // If feed low sensor sees green, Light 2 GREEN
        robot.shotController.isShootingTrigger().negate()
                .and(this.robot.sensors.feedLowSeesGreen)
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light2,
                            IndicatorLightsConstants.IndicatorColors.GREEN);
                }));
        // If feed low sensor sees purple, Light 2 PURPLE
        robot.shotController.isShootingTrigger().negate()
                .and(this.robot.sensors.feedLowSeesPurple)
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light2,
                            IndicatorLightsConstants.IndicatorColors.PURPLE);
                }));
        // If feed low sensor sees neither, Light 2 OFF
        robot.shotController.isShootingTrigger().negate()
                .and(this.robot.sensors.feedLowSeesGreen.negate())
                .and(this.robot.sensors.feedLowSeesPurple.negate())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light2,
                            IndicatorLightsConstants.IndicatorColors.OFF);
                }));

        // Light 3 reflects color of ball seen by feed high sensor, if any
        // If feed high sensor sees green, Light 3 GREEN
        robot.shotController.isShootingTrigger().negate()
                .and(this.robot.sensors.feedHighSeesGreen)
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light3,
                            IndicatorLightsConstants.IndicatorColors.GREEN);
                }));
        // If feed high sensor sees purple, Light 3 PURPLE
        robot.shotController.isShootingTrigger().negate()
                .and(this.robot.sensors.feedHighSeesPurple)
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light3,
                            IndicatorLightsConstants.IndicatorColors.PURPLE);
                }));
        // If feed high sensor sees neither, Light 3 OFF
        robot.shotController.isShootingTrigger().negate()
                .and(this.robot.sensors.feedHighSeesGreen.negate())
                .and(this.robot.sensors.feedHighSeesPurple.negate())
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
