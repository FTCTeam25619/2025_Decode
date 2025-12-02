package org.firstinspires.ftc.teamcode.triggers;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.constants.HoodConstants;
import org.firstinspires.ftc.teamcode.constants.IndicatorLightsConstants;
import org.firstinspires.ftc.teamcode.lib.triggers.ControllerTriggers;
import org.firstinspires.ftc.teamcode.lib.triggers.LeftStickY;
import org.firstinspires.ftc.teamcode.lib.triggers.RightStickY;
import org.firstinspires.ftc.teamcode.lib.triggers.TriggerAxis;
import org.firstinspires.ftc.teamcode.lib.triggers.TriggerButton;
import org.firstinspires.ftc.teamcode.subsystems.IndicatorLights;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class CompetitionTriggers implements ControllerTriggers {
    private final NovaPyraRobot robot;
    private final GamepadEx controller1;
    private final GamepadEx controller2;

    public CompetitionTriggers(NovaPyraRobot robot) {
        this.robot = robot;
        this.controller1 = robot.controller1;
        this.controller2 = robot.controller2;
    }

    public void setupGamepadButtonMappings() {
        setupNonControllerTriggers();
        setupController1ButtonMappings();
        setupController2ButtonMappings();
    }

    public void setupNonControllerTriggers() {
        this.robot.sensors.intakeSeesGreen.whenActive(new InstantCommand(() -> {
            this.robot.indicatorLights.setLightColor(
                    IndicatorLights.Indicators.Light1,
                    IndicatorLightsConstants.IndicatorColors.GREEN);
        }));
        this.robot.sensors.intakeSeesGreen.whenInactive(new InstantCommand(() -> {
            this.robot.indicatorLights.setLightColor(
                    IndicatorLights.Indicators.Light1,
                    IndicatorLightsConstants.IndicatorColors.OFF);
        }));
        this.robot.sensors.intakeSeesPurple.whenActive(new InstantCommand(() -> {
            this.robot.indicatorLights.setLightColor(
                    IndicatorLights.Indicators.Light1,
                    IndicatorLightsConstants.IndicatorColors.PURPLE);
        }));
        this.robot.sensors.intakeSeesPurple.whenInactive(new InstantCommand(() -> {
            this.robot.indicatorLights.setLightColor(
                    IndicatorLights.Indicators.Light1,
                    IndicatorLightsConstants.IndicatorColors.OFF);
        }));
        this.robot.sensors.feedLowSeesGreen.whenActive(new InstantCommand(() -> {
            this.robot.indicatorLights.setLightColor(
                    IndicatorLights.Indicators.Light2,
                    IndicatorLightsConstants.IndicatorColors.GREEN);
        }));
        this.robot.sensors.feedLowSeesGreen.whenInactive(new InstantCommand(() -> {
            this.robot.indicatorLights.setLightColor(
                    IndicatorLights.Indicators.Light2,
                    IndicatorLightsConstants.IndicatorColors.OFF);
        }));
        this.robot.sensors.feedLowSeesPurple.whenActive(new InstantCommand(() -> {
            this.robot.indicatorLights.setLightColor(
                    IndicatorLights.Indicators.Light2,
                    IndicatorLightsConstants.IndicatorColors.PURPLE);
        }));
        this.robot.sensors.feedLowSeesPurple.whenInactive(new InstantCommand(() -> {
            this.robot.indicatorLights.setLightColor(
                    IndicatorLights.Indicators.Light2,
                    IndicatorLightsConstants.IndicatorColors.OFF);
        }));

        this.robot.sensors.feedHighSeesGreen.whenActive(new InstantCommand(() -> {
            this.robot.indicatorLights.setLightColor(
                    IndicatorLights.Indicators.Light3,
                    IndicatorLightsConstants.IndicatorColors.GREEN);
        }));
        this.robot.sensors.feedHighSeesGreen.whenInactive(new InstantCommand(() -> {
            this.robot.indicatorLights.setLightColor(
                    IndicatorLights.Indicators.Light3,
                    IndicatorLightsConstants.IndicatorColors.OFF);
        }));
        this.robot.sensors.feedHighSeesPurple.whenActive(new InstantCommand(() -> {
            this.robot.indicatorLights.setLightColor(
                    IndicatorLights.Indicators.Light3,
                    IndicatorLightsConstants.IndicatorColors.PURPLE);
        }));this.robot.sensors.feedHighSeesPurple.whenInactive(new InstantCommand(() -> {
            this.robot.indicatorLights.setLightColor(
                    IndicatorLights.Indicators.Light3,
                    IndicatorLightsConstants.IndicatorColors.OFF);
        }));
    }

    private void setupController1ButtonMappings() {
        // Setup controller 1 buttons
        GamepadButton c1DPadUp = new GamepadButton(controller1, GamepadKeys.Button.DPAD_UP);
        GamepadButton c1DPadDown = new GamepadButton(controller1, GamepadKeys.Button.DPAD_DOWN);
        GamepadButton c1DPadLeft = new GamepadButton(controller1, GamepadKeys.Button.DPAD_LEFT);
        GamepadButton c1DPadRight = new GamepadButton(controller1, GamepadKeys.Button.DPAD_RIGHT);
        GamepadButton c1A = new GamepadButton(controller1, GamepadKeys.Button.A);
        GamepadButton c1B = new GamepadButton(controller1, GamepadKeys.Button.B);
        GamepadButton c1X = new GamepadButton(controller1, GamepadKeys.Button.X);
        GamepadButton c1Y = new GamepadButton(controller1, GamepadKeys.Button.Y);
        GamepadButton c1LeftBumper = new GamepadButton(controller1, GamepadKeys.Button.LEFT_BUMPER);
        GamepadButton c1RightBumper = new GamepadButton(controller1, GamepadKeys.Button.RIGHT_BUMPER);
        TriggerAxis c1LeftTrigger = new TriggerAxis(controller1, GamepadKeys.Trigger.LEFT_TRIGGER, DEFAULT_AXIS_THRESHOLD);
        TriggerAxis c1RightTrigger = new TriggerAxis(controller1, GamepadKeys.Trigger.RIGHT_TRIGGER, DEFAULT_AXIS_THRESHOLD);
        GamepadButton c1LeftStickPress = new GamepadButton(controller1, GamepadKeys.Button.LEFT_STICK_BUTTON);
        GamepadButton c1RightStickPress = new GamepadButton(controller1, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        GamepadButton c1Back = new GamepadButton(controller1, GamepadKeys.Button.BACK);
        GamepadButton c1Start = new GamepadButton(controller1, GamepadKeys.Button.START);

        // Map controller 1 buttons to commands
        c1LeftTrigger.whileActiveContinuous(new RepeatCommand(new InstantCommand(() -> {
//            robot.shooterFeed.setState(ShooterFeed.FeedState.Feeding);
            robot.intake.setState(Intake.IntakeState.Intake);
        })));
        c1LeftTrigger.and(c1LeftBumper).whileActiveContinuous(new RepeatCommand(new InstantCommand(() -> {
//            robot.shooterFeed.setState(ShooterFeed.FeedState.Dejam);
            robot.intake.setState(Intake.IntakeState.Dejam);
        })));
        c1LeftTrigger.whenInactive(new InstantCommand(() -> {
//            robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
            robot.intake.setState(Intake.IntakeState.Stopped);
        }));
        c1DPadRight.whenPressed(new InstantCommand(() -> robot.shooterFeed.changeFeedPower(0.01)));
        c1DPadLeft.whenPressed(new InstantCommand(() -> robot.shooterFeed.changeFeedPower(-0.01)));

        c1RightTrigger.whenActive(new InstantCommand(() -> {
            robot.shooter.setState(Shooter.ShooterState.Shooting);
        }));
        c1RightTrigger.whenInactive(new InstantCommand(() -> {
            robot.shooter.setState(Shooter.ShooterState.Stopped);
        }));
        c1DPadUp.whenPressed(new InstantCommand(() -> robot.shooter.changeTargetRPM(100)));
        c1DPadDown.whenPressed(new InstantCommand(() -> robot.shooter.changeTargetRPM(-100)));

        c1A.whenPressed(new InstantCommand(() -> robot.hood.moveBy(HoodConstants.Control.incrementSize)));
        c1B.whenPressed(new InstantCommand(() -> robot.hood.moveBy(-HoodConstants.Control.incrementSize)));

        // Reset odometry heading to 0 degrees
        c1Start.whenPressed(new InstantCommand(() -> robot.odometry.setOdometryHeading(0.0, AngleUnit.DEGREES)));
    }

    private void setupController2ButtonMappings() {
        // Setup controller 2 buttons
        LeftStickY c2LeftStickYUp = new LeftStickY(controller2, 0.6);
        LeftStickY c2LeftStickYDown = new LeftStickY(controller2, -0.6);
        RightStickY c2RightStickYUp = new RightStickY(controller2, 0.6);
        RightStickY c2RightStickYDown = new RightStickY(controller2, -0.6);
        GamepadButton c2DPadUp = new GamepadButton(controller2, GamepadKeys.Button.DPAD_UP);
        GamepadButton c2DPadDown = new GamepadButton(controller2, GamepadKeys.Button.DPAD_DOWN);
        GamepadButton c2DPadLeft = new GamepadButton(controller2, GamepadKeys.Button.DPAD_LEFT);
        GamepadButton c2DPadRight = new GamepadButton(controller2, GamepadKeys.Button.DPAD_RIGHT);
        GamepadButton c2A = new GamepadButton(controller2, GamepadKeys.Button.A);
        GamepadButton c2B = new GamepadButton(controller2, GamepadKeys.Button.B);
        GamepadButton c2X = new GamepadButton(controller2, GamepadKeys.Button.X);
        GamepadButton c2Y = new GamepadButton(controller2, GamepadKeys.Button.Y);
        GamepadButton c2LeftBumper = new GamepadButton(controller2, GamepadKeys.Button.LEFT_BUMPER);
        GamepadButton c2RightBumper = new GamepadButton(controller2, GamepadKeys.Button.RIGHT_BUMPER);
        TriggerButton c2LeftTrigger = new TriggerButton(controller2, GamepadKeys.Trigger.LEFT_TRIGGER, 0.6);
        TriggerButton c2RightTrigger = new TriggerButton(controller2, GamepadKeys.Trigger.RIGHT_TRIGGER, 0.6);
        GamepadButton c2LeftStickPress = new GamepadButton(controller2, GamepadKeys.Button.LEFT_STICK_BUTTON);
        GamepadButton c2RightStickPress = new GamepadButton(controller2, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        GamepadButton c2Back = new GamepadButton(controller2, GamepadKeys.Button.BACK);
        GamepadButton c2Start = new GamepadButton(controller2, GamepadKeys.Button.START);
        GamepadButton c1Back = new GamepadButton(controller1, GamepadKeys.Button.BACK);
        GamepadButton c1Start = new GamepadButton(controller1, GamepadKeys.Button.START);

        // Map controller 2 buttons to commands
    }
}
