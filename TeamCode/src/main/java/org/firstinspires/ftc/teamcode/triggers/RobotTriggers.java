package org.firstinspires.ftc.teamcode.triggers;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.constants.IndicatorLightsConstants;
import org.firstinspires.ftc.teamcode.subsystems.IndicatorLights;

public class RobotTriggers {
    private final NovaPyraRobot robot;

    public RobotTriggers(NovaPyraRobot robot) {
        this.robot = robot;
    }

    public void setupNonControllerTriggers() {
        this.robot.sensors.intakeSeesGreen
                .and(robot.shotController.isShootingTrigger().negate())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light1,
                            IndicatorLightsConstants.IndicatorColors.GREEN);
                }));
        this.robot.sensors.intakeSeesGreen
                .and(robot.shotController.isShootingTrigger().negate())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light1,
                            IndicatorLightsConstants.IndicatorColors.OFF);
                }));
        this.robot.sensors.intakeSeesPurple
                .and(robot.shotController.isShootingTrigger().negate())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light1,
                            IndicatorLightsConstants.IndicatorColors.PURPLE);
                }));
        this.robot.sensors.intakeSeesPurple
                .and(robot.shotController.isShootingTrigger().negate())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light1,
                            IndicatorLightsConstants.IndicatorColors.OFF);
                }));
        this.robot.sensors.feedLowSeesGreen
                .and(robot.shotController.isShootingTrigger().negate())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light2,
                            IndicatorLightsConstants.IndicatorColors.GREEN);
                }));
        this.robot.sensors.feedLowSeesGreen
                .and(robot.shotController.isShootingTrigger().negate())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light2,
                            IndicatorLightsConstants.IndicatorColors.OFF);
                }));
        this.robot.sensors.feedLowSeesPurple
                .and(robot.shotController.isShootingTrigger().negate())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light2,
                            IndicatorLightsConstants.IndicatorColors.PURPLE);
                }));
        this.robot.sensors.feedLowSeesPurple
                .and(robot.shotController.isShootingTrigger().negate())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light2,
                            IndicatorLightsConstants.IndicatorColors.OFF);
                }));

        this.robot.sensors.feedHighSeesGreen
                .and(robot.shotController.isShootingTrigger().negate())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light3,
                            IndicatorLightsConstants.IndicatorColors.GREEN);
                }));
        this.robot.sensors.feedHighSeesGreen
                .and(robot.shotController.isShootingTrigger().negate())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light3,
                            IndicatorLightsConstants.IndicatorColors.OFF);
                }));
        this.robot.sensors.feedHighSeesPurple
                .and(robot.shotController.isShootingTrigger().negate())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light3,
                            IndicatorLightsConstants.IndicatorColors.PURPLE);
                }));
        this.robot.sensors.feedHighSeesPurple
                .and(robot.shotController.isShootingTrigger().negate())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light3,
                            IndicatorLightsConstants.IndicatorColors.OFF);
                }));

        this.robot.shotController.isShootingTrigger()
                .and(this.robot.shotController.shooterAtRPM().negate())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light1,
                            IndicatorLightsConstants.IndicatorColors.RED);
                }));
        this.robot.shotController.isShootingTrigger()
                .and(this.robot.shotController.shooterAtRPM())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light1,
                            IndicatorLightsConstants.IndicatorColors.GREEN);
                }));
        this.robot.shotController.isShootingTrigger()
                .and(this.robot.shotController.readyToShootTrigger().negate())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light2,
                            IndicatorLightsConstants.IndicatorColors.RED);
                }));
        this.robot.shotController.isShootingTrigger()
                .and(this.robot.shotController.readyToShootTrigger())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light2,
                            IndicatorLightsConstants.IndicatorColors.GREEN);
                }));
        this.robot.shotController.isShootingTrigger()
                .and(this.robot.shotController.robotAtHeading().negate())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light3,
                            IndicatorLightsConstants.IndicatorColors.RED);
                }));
        this.robot.shotController.isShootingTrigger()
                .and(this.robot.shotController.robotAtHeading())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light3,
                            IndicatorLightsConstants.IndicatorColors.GREEN);
                }));

        this.robot.shotController.setupStateTransitionTriggers();
    }
}
