package org.firstinspires.ftc.teamcode.triggers;

import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.constants.HoodConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterFeedConstants;
import org.firstinspires.ftc.teamcode.lib.triggers.ControllerTriggers;
import org.firstinspires.ftc.teamcode.lib.triggers.LeftStickY;
import org.firstinspires.ftc.teamcode.lib.triggers.RightStickY;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeed;

public class TuningTriggers implements ControllerTriggers {
    private final NovaPyraRobot robot;
    private final GamepadEx controller1;
    private final GamepadEx controller2;

    public TuningTriggers(NovaPyraRobot robot, GamepadEx controller1, GamepadEx controller2) {
        this.robot = robot;
        this.controller1 = controller1;
        this.controller2 = controller2;
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
        GamepadButton c2DPadUp = new GamepadButton(controller2, GamepadKeys.Button.DPAD_UP);
        GamepadButton c2DPadDown = new GamepadButton(controller2, GamepadKeys.Button.DPAD_DOWN);
        GamepadButton c2DPadLeft = new GamepadButton(controller2, GamepadKeys.Button.DPAD_LEFT);
        GamepadButton c2DPadRight = new GamepadButton(controller2, GamepadKeys.Button.DPAD_RIGHT);
        GamepadButton c2LeftBumper = new GamepadButton(controller2, GamepadKeys.Button.LEFT_BUMPER);
        GamepadButton c2RightBumper = new GamepadButton(controller2, GamepadKeys.Button.RIGHT_BUMPER);
        GamepadButton c2Back = new GamepadButton(controller2, GamepadKeys.Button.BACK);
        GamepadButton c2Start = new GamepadButton(controller2, GamepadKeys.Button.START);

        c2LeftStickYUp.whenActive(robot.lift::engage);
        c2LeftStickYDown.whenActive(robot.lift::disengage);
        c2RightStickYUp.whenActive(() -> {
            robot.shooterFeed.setFeedPower(ShooterFeedConstants.Targets.defaultFeedPower);
            robot.shooterFeed.setState(ShooterFeed.FeedState.Feeding);
        });
        c2A.whenActive(robot.feedStop::engage);
        c2X.whenActive(robot.feedStop::disengage);
        c2B.whenActive(() -> robot.hood.setAngle(HoodConstants.Targets.farWallShot));
        c2Y.whenActive(() -> robot.hood.setAngle(HoodConstants.Targets.atGoalShot));
        c2LeftBumper.whenActive(() -> robot.intake.setState(Intake.IntakeState.Intake));
        c2RightBumper.whenActive(() -> {
            robot.shooter.setTargetRPM(ShooterConstants.Targets.idleRPM);
            robot.shooter.setState(Shooter.ShooterState.Shooting);
        });
        c2DPadUp.whenActive(() -> robot.shooter.setTargetRPM(ShooterConstants.Targets.farWallShot));
        c2DPadRight.whenActive(() -> robot.shooter.setTargetRPM(ShooterConstants.Targets.nearZoneEndShot));
        c2DPadLeft.whenActive(() -> robot.shooter.setTargetRPM(ShooterConstants.Targets.nearZoneMidShot));
        c2DPadDown.whenActive(() -> robot.shooter.setTargetRPM(ShooterConstants.Targets.atGoalShot));
        c2Back.whenActive(() -> {
            robot.shooter.setState(Shooter.ShooterState.Stopped);
            robot.intake.setState(Intake.IntakeState.Stopped);
            robot.shooterFeed.setState(ShooterFeed.FeedState.Stopped);
        });
    }
}
