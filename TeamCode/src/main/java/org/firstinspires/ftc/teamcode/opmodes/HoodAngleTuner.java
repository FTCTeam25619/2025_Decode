package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.constants.HoodConstants;
import org.firstinspires.ftc.teamcode.subsystems.Hood;

/**
 * Simple OpMode for tuning hood angle using your existing Hood subsystem
 *
 * Controls:
 * - D-Pad Up: Increase hood angle by increment size
 * - D-Pad Down: Decrease hood angle by increment size
 * - Left Bumper: Increase hood angle by 1 degree (fine tune)
 * - Right Bumper: Decrease hood angle by 1 degree (fine tune)
 * - A: Set to minimum angle
 * - B: Set to maximum angle
 * - X: Set to middle position
 * - Y: Set to current default/preset angle
 */
@TeleOp(name = "Hood Angle Tuner", group = "Tuning")
public class HoodAngleTuner extends CommandOpMode {

    private Hood hood;
    private GamepadEx gamepadEx;

    // Current target angle
    private double targetAngle;

    @Override
    public void initialize() {
        super.reset();

        // Initialize hood subsystem
        hood = new Hood(hardwareMap, telemetry);

        // Start at middle position
        targetAngle = (HoodConstants.Limits.hoodHardStopMin + HoodConstants.Limits.hoodHardStopMax) / 2.0;
        hood.setAngle(targetAngle);

        // Setup gamepad
        gamepadEx = new GamepadEx(gamepad1);
        setupButtonBindings();

        telemetry.addLine("Hood Angle Tuner Ready");
        telemetry.addLine();
        telemetry.addData("Range", "%.0f° to %.0f°",
                HoodConstants.Limits.hoodHardStopMin,
                HoodConstants.Limits.hoodHardStopMax);
        telemetry.addData("Increment Size", "%.1f°", HoodConstants.Control.incrementSize);
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("D-Pad Up/Down: ±" + HoodConstants.Control.incrementSize + "°");
        telemetry.addLine("LB/RB: ±1° (fine)");
        telemetry.addLine("A: Min | B: Max | X: Mid | Y: Preset");
        telemetry.update();
    }

    private void setupButtonBindings() {
        // Create button objects
        GamepadButton dpadUp = new GamepadButton(gamepadEx, GamepadKeys.Button.DPAD_UP);
        GamepadButton dpadDown = new GamepadButton(gamepadEx, GamepadKeys.Button.DPAD_DOWN);
        GamepadButton leftBumper = new GamepadButton(gamepadEx, GamepadKeys.Button.LEFT_BUMPER);
        GamepadButton rightBumper = new GamepadButton(gamepadEx, GamepadKeys.Button.RIGHT_BUMPER);
        GamepadButton buttonA = new GamepadButton(gamepadEx, GamepadKeys.Button.A);
        GamepadButton buttonB = new GamepadButton(gamepadEx, GamepadKeys.Button.B);
        GamepadButton buttonX = new GamepadButton(gamepadEx, GamepadKeys.Button.X);
        GamepadButton buttonY = new GamepadButton(gamepadEx, GamepadKeys.Button.Y);

        // D-Pad Up - Increase by increment size
        dpadUp.whenPressed(new InstantCommand(() -> {
            targetAngle += HoodConstants.Control.incrementSize;
            hood.setAngle(targetAngle);
            targetAngle = clampAngle(targetAngle);
        }));

        // D-Pad Down - Decrease by increment size
        dpadDown.whenPressed(new InstantCommand(() -> {
            targetAngle -= HoodConstants.Control.incrementSize;
            hood.setAngle(targetAngle);
            targetAngle = clampAngle(targetAngle);
        }));

        // Left Bumper - Fine increase by 1 degree
        leftBumper.whenPressed(new InstantCommand(() -> {
            targetAngle += 1.0;
            hood.setAngle(targetAngle);
            targetAngle = clampAngle(targetAngle);
        }));

        // Right Bumper - Fine decrease by 1 degree
        rightBumper.whenPressed(new InstantCommand(() -> {
            targetAngle -= 1.0;
            hood.setAngle(targetAngle);
            targetAngle = clampAngle(targetAngle);
        }));

        // A Button - Set to minimum
        buttonA.whenPressed(new InstantCommand(() -> {
            targetAngle = HoodConstants.Limits.hoodHardStopMin;
            hood.setAngle(targetAngle);
        }));

        // B Button - Set to maximum
        buttonB.whenPressed(new InstantCommand(() -> {
            targetAngle = HoodConstants.Limits.hoodHardStopMax;
            hood.setAngle(targetAngle);
        }));

        // X Button - Set to middle
        buttonX.whenPressed(new InstantCommand(() -> {
            targetAngle = (HoodConstants.Limits.hoodHardStopMin + HoodConstants.Limits.hoodHardStopMax) / 2.0;
            hood.setAngle(targetAngle);
        }));

        // Y Button - Set to a preset (AtGoal as example)
        buttonY.whenPressed(new InstantCommand(() -> {
            targetAngle = HoodConstants.Targets.atGoalShot;
            hood.setAngle(targetAngle);
        }));
    }

    @Override
    public void run() {
        super.run();

        // Update hood subsystem
        hood.periodic();

        // Display telemetry
        telemetry.addData("═══ HOOD ANGLE ═══", "");
        telemetry.addData("Current Angle", "%.1f°", targetAngle);
        telemetry.addData("Range", "%.0f° to %.0f°",
                HoodConstants.Limits.hoodHardStopMin,
                HoodConstants.Limits.hoodHardStopMax);
        telemetry.addLine();

        telemetry.addData("═══ PRESETS ═══", "");
        telemetry.addData("At Goal", "%.1f°", HoodConstants.Targets.atGoalShot);
        telemetry.addData("Near Zone Mid", "%.1f°", HoodConstants.Targets.nearZoneMidShot);
        telemetry.addData("Near Zone End", "%.1f°", HoodConstants.Targets.nearZoneEndShot);
        telemetry.addData("Far Wall", "%.1f°", HoodConstants.Targets.farWallShot);
        telemetry.addLine();

        telemetry.addData("═══ CONTROLS ═══", "");
        telemetry.addLine("D-Pad Up/Down: ±" + HoodConstants.Control.incrementSize + "°");
        telemetry.addLine("LB/RB: ±1° (fine)");
        telemetry.addLine("A: Min | B: Max | X: Mid");
        telemetry.addLine("Y: AtGoal Preset");

        telemetry.update();
    }

    /**
     * Clamp angle to valid range and return the clamped value
     */
    private double clampAngle(double angle) {
        return Math.max(HoodConstants.Limits.hoodHardStopMin,
                Math.min(HoodConstants.Limits.hoodHardStopMax, angle));
    }

    @Override
    public void reset() {
        // Nothing to clean up
    }
}