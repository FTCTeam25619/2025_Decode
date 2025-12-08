package org.firstinspires.ftc.teamcode.triggers;

import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.lib.triggers.ControllerTriggers;
import org.firstinspires.ftc.teamcode.lib.triggers.LeftStickY;
import org.firstinspires.ftc.teamcode.lib.triggers.RightStickY;

public class TuningTriggers implements ControllerTriggers {
    private final NovaPyraRobot robot;
    private final GamepadEx controller1;
    private final GamepadEx controller2;

    public TuningTriggers(NovaPyraRobot robot) {
        this.robot = robot;
        this.controller1 = robot.controller1;
        this.controller2 = robot.controller2;
    }

    public void setupGamepadButtonMappings() {
        setupController1ButtonMappings();
        setupController2ButtonMappings();
    }

    private void setupController1ButtonMappings() {
        GamepadButton c1A = new GamepadButton(controller1, GamepadKeys.Button.A);
        GamepadButton c1B = new GamepadButton(controller1, GamepadKeys.Button.B);
        GamepadButton c1X = new GamepadButton(controller1, GamepadKeys.Button.X);
        GamepadButton c1Y = new GamepadButton(controller1, GamepadKeys.Button.Y);
        GamepadButton c1LeftBumper = new GamepadButton(controller1, GamepadKeys.Button.LEFT_BUMPER);
        GamepadButton c1RightBumper = new GamepadButton(controller1, GamepadKeys.Button.RIGHT_BUMPER);
        GamepadButton c1Back = new GamepadButton(controller1, GamepadKeys.Button.BACK);
        GamepadButton c1Start = new GamepadButton(controller1, GamepadKeys.Button.START);
        // Test positions for tuning
    }

    private void setupController2ButtonMappings() {
        LeftStickY c2LeftStickYUp = new LeftStickY(controller2, DEFAULT_AXIS_THRESHOLD);
        LeftStickY c2LeftStickYDown = new LeftStickY(controller2, -DEFAULT_AXIS_THRESHOLD);
        RightStickY c2RightStickYUp = new RightStickY(controller2, DEFAULT_AXIS_THRESHOLD);
        RightStickY c2RightStickYDown = new RightStickY(controller2, -DEFAULT_AXIS_THRESHOLD);
        GamepadButton c2A = new GamepadButton(controller2, GamepadKeys.Button.A);
        GamepadButton c2B = new GamepadButton(controller2, GamepadKeys.Button.B);
        GamepadButton c2X = new GamepadButton(controller2, GamepadKeys.Button.X);
        GamepadButton c2Y = new GamepadButton(controller2, GamepadKeys.Button.Y);
        GamepadButton c2LeftBumper = new GamepadButton(controller2, GamepadKeys.Button.LEFT_BUMPER);
        GamepadButton c2Back = new GamepadButton(controller2, GamepadKeys.Button.BACK);
        GamepadButton c2Start = new GamepadButton(controller2, GamepadKeys.Button.START);
    }
}
