package org.firstinspires.ftc.teamcode.triggers;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.constants.IndicatorLightsConstants;
import org.firstinspires.ftc.teamcode.lib.orchestration.PresetShot;
import org.firstinspires.ftc.teamcode.lib.orchestration.ShotCycleState;
import org.firstinspires.ftc.teamcode.lib.triggers.ControllerTriggers;
import org.firstinspires.ftc.teamcode.lib.triggers.LeftStickY;
import org.firstinspires.ftc.teamcode.lib.triggers.RightStickY;
import org.firstinspires.ftc.teamcode.lib.triggers.TriggerAxis;
import org.firstinspires.ftc.teamcode.lib.triggers.TriggerButton;
import org.firstinspires.ftc.teamcode.subsystems.IndicatorLights;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

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
        this.robot.sensors.intakeSeesGreen
                .and(robot.shotController.isShootingTrigger().negate())
                .whenActive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light1,
                            IndicatorLightsConstants.IndicatorColors.GREEN);
                }));
        this.robot.sensors.intakeSeesGreen
                .and(robot.shotController.isShootingTrigger().negate())
                .whenInactive(new InstantCommand(() -> {
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
                .whenInactive(new InstantCommand(() -> {
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
                .whenInactive(new InstantCommand(() -> {
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
                .whenInactive(new InstantCommand(() -> {
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
                .whenInactive(new InstantCommand(() -> {
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
                .whenInactive(new InstantCommand(() -> {
                    this.robot.indicatorLights.setLightColor(
                            IndicatorLights.Indicators.Light3,
                            IndicatorLightsConstants.IndicatorColors.OFF);
                }));

        // While in shooting pipeline, use lights to indicate RPM/ready/heading
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

        // LEFT TRIGGER: INTAKE
        c1LeftTrigger.whenActive(new InstantCommand(() -> {
            robot.shotController.setState(ShotCycleState.INTAKE);
        }));

        // On release of LT → go back to IDLE (ShotController will auto-STAGED if a piece is detected)
        c1LeftTrigger.whenInactive(new InstantCommand(() -> {
            robot.shotController.setState(ShotCycleState.IDLE);
        }));

        // Dejam while both LT + LB held
        c1LeftTrigger.and(c1LeftBumper).whileActiveContinuous(
                new RepeatCommand(new InstantCommand(() -> {
                    robot.intake.setState(Intake.IntakeState.Dejam);
                })));

        // LEFT BUMPER: PREPARE SHOT (aim & spin up while driving)
        c1LeftBumper.whenActive(new InstantCommand(() -> {
            // Only make sense to prepare if we actually have a staged piece
            if (robot.shotController.staged()) {
                robot.shotController.setState(ShotCycleState.PREPARE_SHOT);
            }
        }));

        c1LeftBumper.whenInactive(new InstantCommand(() -> {
            // If we were in any shooting-related state, drop back to IDLE
            if (robot.shotController.isShooting()) {
                robot.shotController.setState(ShotCycleState.IDLE);
            }
        }));

        // RIGHT TRIGGER: SHOOT (only when READY_TO_SHOOT)
        c1RightTrigger.whenActive(new InstantCommand(() -> {
            if (robot.shotController.getState() == ShotCycleState.READY_TO_SHOOT) {
                robot.shotController.setState(ShotCycleState.SHOOTING);
            }
        }));

        c1RightTrigger.whenInactive(new InstantCommand(() -> {
            // After shot or abort, if LB still held, go back to PREPARE_SHOT; else IDLE
            if (controller1.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                if (robot.shotController.staged()) {
                    robot.shotController.setState(ShotCycleState.PREPARE_SHOT);
                } else {
                    robot.shotController.setState(ShotCycleState.IDLE);
                }
            } else {
                robot.shotController.setState(ShotCycleState.IDLE);
            }
        }));


        // PRESET SHOTS (fallback / manual)
        c1A.whenPressed(new InstantCommand(() -> {
            robot.shotController.setupPresetShot(PresetShot.AtGoal);
        }));
        c1X.whenPressed(new InstantCommand(() -> {
            robot.shotController.setupPresetShot(PresetShot.NearZoneMid);
        }));
        c1B.whenPressed(new InstantCommand(() -> {
            robot.shotController.setupPresetShot(PresetShot.NearZoneEnd);
        }));
        c1Y.whenPressed(new InstantCommand(() -> {
            robot.shotController.setupPresetShot(PresetShot.FarWall);
        }));

        // Shooter RPM fine-tune
        c1DPadUp.whenPressed(new InstantCommand(() -> robot.shooter.changeTargetRPM(100)));
        c1DPadDown.whenPressed(new InstantCommand(() -> robot.shooter.changeTargetRPM(-100)));

        // Shooter feed power fine-tune
        c1DPadRight.whenPressed(new InstantCommand(() -> robot.shooterFeed.changeFeedPower(0.01)));
        c1DPadLeft.whenPressed(new InstantCommand(() -> robot.shooterFeed.changeFeedPower(-0.01)));

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
