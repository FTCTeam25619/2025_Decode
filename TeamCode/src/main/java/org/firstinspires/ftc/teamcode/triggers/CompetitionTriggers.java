package org.firstinspires.ftc.teamcode.triggers;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.lib.orchestration.PresetShot;
import org.firstinspires.ftc.teamcode.lib.orchestration.ShotCycleState;
import org.firstinspires.ftc.teamcode.lib.triggers.ControllerTriggers;
import org.firstinspires.ftc.teamcode.lib.triggers.LeftStickY;
import org.firstinspires.ftc.teamcode.lib.triggers.RightStickY;
import org.firstinspires.ftc.teamcode.lib.triggers.TriggerAxis;
import org.firstinspires.ftc.teamcode.lib.triggers.TriggerButton;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class CompetitionTriggers implements ControllerTriggers {
    private final NovaPyraRobot robot;
    private final GamepadEx controller1;
    private final GamepadEx controller2;

    public CompetitionTriggers(NovaPyraRobot robot, GamepadEx controller1, GamepadEx controller2) {
        this.robot = robot;
        this.controller1 = controller1;
        this.controller2 = controller2;
    }

    public void setupGamepadButtonMappings() {
        setupController1ButtonMappings();
        setupController2ButtonMappings();
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
        c1LeftTrigger.whenActive(new InstantCommand(() -> {
            robot.shotController.setState(ShotCycleState.INTAKE);
        }));
        c1LeftTrigger.and(c1LeftBumper).whileActiveContinuous(new RepeatCommand(new InstantCommand(() -> {
            robot.intake.setState(Intake.IntakeState.Dejam);
        })));
        c1LeftTrigger.whenInactive(new InstantCommand(() -> {
            robot.shotController.setState(ShotCycleState.IDLE);
        }));
        c1DPadRight.whenPressed(new InstantCommand(() -> robot.shooterFeed.changeFeedPower(0.01)));
        c1DPadLeft.whenPressed(new InstantCommand(() -> robot.shooterFeed.changeFeedPower(-0.01)));

        c1RightTrigger.and(c1RightBumper.negate()).whenActive(new InstantCommand(() -> {
            robot.shotController.setState(ShotCycleState.PREPARE_SHOT);
        }));
        c1RightBumper.whenActive(new InstantCommand(() -> {
            robot.shotController.setState(ShotCycleState.SHOOTING);
        }));
        c1RightBumper.negate().and(robot.shotController.presetShotMode()).whenActive(
                new InstantCommand(() -> robot.shotController.setState(ShotCycleState.IDLE)));
        c1RightTrigger.whenInactive(new InstantCommand(() -> {
            robot.shotController.setState(ShotCycleState.IDLE);
        }));
        c1DPadUp.whenPressed(new InstantCommand(() -> robot.shooter.changeTargetRPM(100)));
        c1DPadDown.whenPressed(new InstantCommand(() -> robot.shooter.changeTargetRPM(-100)));

//        c1A.whenPressed(new InstantCommand(() -> robot.hood.moveBy(HoodConstants.Control.incrementSize)));
//        c1B.whenPressed(new InstantCommand(() -> robot.hood.moveBy(-HoodConstants.Control.incrementSize)));

//        c1RightBumper.whenPressed(new InstantCommand(robot.feedStop::setTestPosition));
//        c1X.whenPressed(new InstantCommand(robot.feedStop::disengage));
//        c1Y.whenPressed(new InstantCommand(robot.feedStop::engage));

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

        // Reset odometry heading to 0 degrees
        c1Start.whenPressed(new InstantCommand(() -> robot.odometry.setOdometryHeading(
                robot.allianceDriveHeadingOffset(), AngleUnit.DEGREES)));
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
