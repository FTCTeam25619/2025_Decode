package org.firstinspires.ftc.teamcode.triggers;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.constants.OdometryConstants;
import org.firstinspires.ftc.teamcode.lib.orchestration.PresetShot;
import org.firstinspires.ftc.teamcode.lib.orchestration.ShotCycleState;
import org.firstinspires.ftc.teamcode.lib.triggers.ControllerTriggers;
import org.firstinspires.ftc.teamcode.lib.triggers.LeftStickY;
import org.firstinspires.ftc.teamcode.lib.triggers.RightStickY;
import org.firstinspires.ftc.teamcode.lib.triggers.TriggerAxis;
import org.firstinspires.ftc.teamcode.lib.triggers.TriggerButton;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * ============================================================================
 * COMPETITION CONTROLLER MAPPINGS
 * ============================================================================
 *
 * CONTROLLER 1 (DRIVER) - Quick Reference:
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │ DRIVING                                                                 │
 * │   Left Stick      → Drive (Forward/Strafe)                              │
 * │   Right Stick     → Manual Rotation (scaled) / Auto-aim when shooting   │
 * │   Right Stick Btn → Force Auto-aim (hold)                               │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ INTAKE & SHOOTING                                                       │
 * │   Left Trigger    → Intake (hold)                                       │
 * │   Left Bumper     → Prepare Shot (spin up, aim)                         │
 * │   Right Trigger   → Shoot (when ready)                                  │
 * │   LT + LB         → Dejam (hold both)                                   │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ SHOT MODE SELECTION                                                     │
 * │   DPad Left       → Auto-Shoot Mode (LUT targeting) [Light4 GREEN]      │
 * │   Back + A        → Preset: At Goal                 [Light4 RED]        │
 * │   Back + X        → Preset: Near Zone Mid           [Light4 RED]        │
 * │   Back + B        → Preset: Near Zone End           [Light4 RED]        │
 * │   Back + Y        → Preset: Far Wall                [Light4 RED]        │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ SHOOTER CONTROL                                                         │
 * │   DPad Up         → Shooter Idle (safe mode)                            │
 * │   DPad Down       → Shooter Off                                         │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ ENDGAME                                                                 │
 * │   Back + RB       → Lift Engage                                         │
 * │   Back + LB       → Lift Disengage                                      │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ UTILITY                                                                 │
 * │   Start           → Reset Odometry Heading                              │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ UNUSED BUTTONS (Available for reassignment)                             │
 * │   A, B, X, Y (without Back)                                             │
 * │   Right Bumper (without Back)                                           │
 * │   DPad Right                                                            │
 * │   Left Stick Button                                                     │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * CONTROLLER 2 (OPERATOR) - Quick Reference:
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │ ALL BUTTONS CURRENTLY UNASSIGNED                                        │
 * │ Available for mechanism control, manual adjustments, etc.               │
 * └─────────────────────────────────────────────────────────────────────────┘
 */
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

    /** ═════════════════════════════════════════════════════════════════════════
     *             CONTROLLER 1 - DRIVER
     * ═════════════════════════════════════════════════════════════════════════
    */

    private void setupController1ButtonMappings() {

        //─────────────────────────────────────────────────────────────────────
        // BUTTON INITIALIZATION
        //─────────────────────────────────────────────────────────────────────

        // D-Pad
        GamepadButton c1DPadUp    = new GamepadButton(controller1, GamepadKeys.Button.DPAD_UP);
        GamepadButton c1DPadDown  = new GamepadButton(controller1, GamepadKeys.Button.DPAD_DOWN);
        GamepadButton c1DPadLeft  = new GamepadButton(controller1, GamepadKeys.Button.DPAD_LEFT);
        GamepadButton c1DPadRight = new GamepadButton(controller1, GamepadKeys.Button.DPAD_RIGHT);

        // Face Buttons
        GamepadButton c1A = new GamepadButton(controller1, GamepadKeys.Button.A);
        GamepadButton c1B = new GamepadButton(controller1, GamepadKeys.Button.B);
        GamepadButton c1X = new GamepadButton(controller1, GamepadKeys.Button.X);
        GamepadButton c1Y = new GamepadButton(controller1, GamepadKeys.Button.Y);

        // Bumpers & Triggers
        GamepadButton c1LeftBumper  = new GamepadButton(controller1, GamepadKeys.Button.LEFT_BUMPER);
        GamepadButton c1RightBumper = new GamepadButton(controller1, GamepadKeys.Button.RIGHT_BUMPER);
        TriggerAxis c1LeftTrigger   = new TriggerAxis(controller1, GamepadKeys.Trigger.LEFT_TRIGGER, DEFAULT_AXIS_THRESHOLD);
        TriggerAxis c1RightTrigger  = new TriggerAxis(controller1, GamepadKeys.Trigger.RIGHT_TRIGGER, DEFAULT_AXIS_THRESHOLD);

        // Stick Buttons
        GamepadButton c1LeftStickPress  = new GamepadButton(controller1, GamepadKeys.Button.LEFT_STICK_BUTTON);
        GamepadButton c1RightStickPress = new GamepadButton(controller1, GamepadKeys.Button.RIGHT_STICK_BUTTON);

        // Utility Buttons
        GamepadButton c1Back  = new GamepadButton(controller1, GamepadKeys.Button.BACK);
        GamepadButton c1Start = new GamepadButton(controller1, GamepadKeys.Button.START);


        //═════════════════════════════════════════════════════════════════════
        // INTAKE CONTROLS
        //═════════════════════════════════════════════════════════════════════

        //─────────────────────────────────────────────────────────────────────
        // LEFT TRIGGER → Intake (hold to run intake)
        //─────────────────────────────────────────────────────────────────────
        c1LeftTrigger.whenActive(new InstantCommand(() -> {
            robot.shotController.setState(ShotCycleState.INTAKE);
        }));

        c1LeftTrigger.whenInactive(new InstantCommand(() -> {
            robot.shotController.setState(ShotCycleState.IDLE);
        }));

        //─────────────────────────────────────────────────────────────────────
        // LEFT TRIGGER + LEFT BUMPER → Dejam (emergency unjam)
        //─────────────────────────────────────────────────────────────────────
        c1LeftTrigger.and(c1LeftBumper).whileActiveContinuous(
                new RepeatCommand(new InstantCommand(() -> {
                    robot.intake.setState(Intake.IntakeState.Dejam);
                }))
        );


        //═════════════════════════════════════════════════════════════════════
        // SHOOTING CONTROLS
        //═════════════════════════════════════════════════════════════════════

        //─────────────────────────────────────────────────────────────────────
        // LEFT BUMPER → Prepare Shot (spin up shooter, aim, keep driving)
        //─────────────────────────────────────────────────────────────────────
        c1LeftBumper.whenActive(new InstantCommand(() -> {
            robot.shotController.setState(ShotCycleState.PREPARE_SHOT);
        }));

        c1LeftBumper.whenInactive(new InstantCommand(() -> {
            // Only return to IDLE if right trigger is also released
            if (controller1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < DEFAULT_AXIS_THRESHOLD) {
                if (robot.shotController.doAutoHeading()) {
                    robot.shotController.setState(ShotCycleState.IDLE);
                }
            }
        }));

        //─────────────────────────────────────────────────────────────────────
        // RIGHT TRIGGER → Shoot (force shoot if held)
        //─────────────────────────────────────────────────────────────────────
        c1RightTrigger.whenActive(new InstantCommand(() -> {
            if (robot.shotController.getState() == ShotCycleState.READY_TO_SHOOT) {
                robot.shotController.setState(ShotCycleState.SHOOTING);
            }
        }));

        c1RightTrigger.whenInactive(new InstantCommand(() -> {
            // Return to appropriate state after shooting
            if (controller1.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                // If still holding LB, go back to preparing
                if (robot.shotController.staged()) {
                    robot.shotController.setState(ShotCycleState.PREPARE_SHOT);
                } else {
                    robot.shotController.setState(ShotCycleState.IDLE);
                }
            } else {
                robot.shotController.setState(ShotCycleState.IDLE);
            }
        }));


        //═════════════════════════════════════════════════════════════════════
        // SHOT MODE SELECTION
        //═════════════════════════════════════════════════════════════════════

        //─────────────────────────────────────────────────────────────────────
        // DPAD LEFT → Return to Auto-Shoot Mode
        // Uses LUT (lookup table) for distance-based targeting
        // Light 4: GREEN
        //─────────────────────────────────────────────────────────────────────
        c1DPadLeft.whenPressed(new InstantCommand(() -> {
            robot.shotController.autoTargetShot();
            // Sets presetShotMode = false → triggers Light4 GREEN
        }));

        //─────────────────────────────────────────────────────────────────────
        // BACK + A → Preset Shot: At Goal
        // Light 4: RED
        //─────────────────────────────────────────────────────────────────────
        c1Back.and(c1A).whenActive(new InstantCommand(() -> {
            robot.shotController.setupPresetShot(PresetShot.AtGoal);
            // Sets presetShotMode = true → triggers Light4 RED
        }));

        //─────────────────────────────────────────────────────────────────────
        // BACK + X → Preset Shot: Near Zone Mid
        // Light 4: RED
        //─────────────────────────────────────────────────────────────────────
        c1Back.and(c1X).whenActive(new InstantCommand(() -> {
            robot.shotController.setupPresetShot(PresetShot.NearZoneMid);
        }));

        //─────────────────────────────────────────────────────────────────────
        // BACK + B → Preset Shot: Near Zone End
        // Light 4: RED
        //─────────────────────────────────────────────────────────────────────
        c1Back.and(c1B).whenActive(new InstantCommand(() -> {
            robot.shotController.setupPresetShot(PresetShot.NearZoneEnd);
        }));

        //─────────────────────────────────────────────────────────────────────
        // BACK + Y → Preset Shot: Far Wall
        // Light 4: RED
        //─────────────────────────────────────────────────────────────────────
        c1Back.and(c1Y).whenActive(new InstantCommand(() -> {
            robot.shotController.setupPresetShot(PresetShot.FarWall);
        }));


        //═════════════════════════════════════════════════════════════════════
        // SHOOTER STATE CONTROL
        //═════════════════════════════════════════════════════════════════════

        //─────────────────────────────────────────────────────────────────────
        // DPAD UP → Shooter Idle (safe, spinning at idle speed)
        //─────────────────────────────────────────────────────────────────────
        c1DPadUp.whenPressed(new InstantCommand(() ->
                robot.shotController.setState(ShotCycleState.IDLE)
        ));

        //─────────────────────────────────────────────────────────────────────
        // DPAD DOWN → Shooter Off (completely stopped)
        //─────────────────────────────────────────────────────────────────────
        c1DPadDown.whenPressed(new InstantCommand(() ->
                robot.shotController.setState(ShotCycleState.OFF)
        ));


        //═════════════════════════════════════════════════════════════════════
        // ENDGAME CONTROLS
        //═════════════════════════════════════════════════════════════════════

        //─────────────────────────────────────────────────────────────────────
        // BACK + RIGHT BUMPER → Engage Lift (climb)
        //─────────────────────────────────────────────────────────────────────
        c1Back.and(c1RightBumper).whenActive(new InstantCommand(robot.lift::engage));

        //─────────────────────────────────────────────────────────────────────
        // BACK + LEFT BUMPER → Disengage Lift (lower)
        //─────────────────────────────────────────────────────────────────────
        c1Back.and(c1LeftBumper).whenActive(new InstantCommand(robot.lift::disengage));


        //═════════════════════════════════════════════════════════════════════
        // UTILITY CONTROLS
        //═════════════════════════════════════════════════════════════════════

        //─────────────────────────────────────────────────────────────────────
        // START → Reset Odometry Heading (resets to alliance-specific heading)
        //─────────────────────────────────────────────────────────────────────
        c1Start.whenPressed(new InstantCommand(() ->
                robot.odometry.setOdometryHeading(
                        OdometryConstants.DefaultStartPose.allianceDriveHeadingOffset(robot.alliance),
                        AngleUnit.DEGREES
                )
        ));


        //═════════════════════════════════════════════════════════════════════
        // UNASSIGNED BUTTONS (Available for new features)
        //═════════════════════════════════════════════════════════════════════
        //
        // Individual face buttons (A, B, X, Y without Back modifier)
        // Right Bumper (without Back modifier)
        // DPad Right
        // Left Stick Button
        // Right Stick Button (currently used for forced auto-aim in Exploration.java)
        //
        //═════════════════════════════════════════════════════════════════════
    }


    /** ═════════════════════════════════════════════════════════════════════════
     *             CONTROLLER 2
     * ═════════════════════════════════════════════════════════════════════════
     */

    private void setupController2ButtonMappings() {

        //─────────────────────────────────────────────────────────────────────
        // BUTTON INITIALIZATION
        //─────────────────────────────────────────────────────────────────────

        // Stick Triggers
        LeftStickY c2LeftStickYUp    = new LeftStickY(controller2, 0.6);
        LeftStickY c2LeftStickYDown  = new LeftStickY(controller2, -0.6);
        RightStickY c2RightStickYUp  = new RightStickY(controller2, 0.6);
        RightStickY c2RightStickYDown = new RightStickY(controller2, -0.6);

        // D-Pad
        GamepadButton c2DPadUp    = new GamepadButton(controller2, GamepadKeys.Button.DPAD_UP);
        GamepadButton c2DPadDown  = new GamepadButton(controller2, GamepadKeys.Button.DPAD_DOWN);
        GamepadButton c2DPadLeft  = new GamepadButton(controller2, GamepadKeys.Button.DPAD_LEFT);
        GamepadButton c2DPadRight = new GamepadButton(controller2, GamepadKeys.Button.DPAD_RIGHT);

        // Face Buttons
        GamepadButton c2A = new GamepadButton(controller2, GamepadKeys.Button.A);
        GamepadButton c2B = new GamepadButton(controller2, GamepadKeys.Button.B);
        GamepadButton c2X = new GamepadButton(controller2, GamepadKeys.Button.X);
        GamepadButton c2Y = new GamepadButton(controller2, GamepadKeys.Button.Y);

        // Bumpers & Triggers
        GamepadButton c2LeftBumper   = new GamepadButton(controller2, GamepadKeys.Button.LEFT_BUMPER);
        GamepadButton c2RightBumper  = new GamepadButton(controller2, GamepadKeys.Button.RIGHT_BUMPER);
        TriggerButton c2LeftTrigger  = new TriggerButton(controller2, GamepadKeys.Trigger.LEFT_TRIGGER, 0.6);
        TriggerButton c2RightTrigger = new TriggerButton(controller2, GamepadKeys.Trigger.RIGHT_TRIGGER, 0.6);

        // Stick Buttons
        GamepadButton c2LeftStickPress  = new GamepadButton(controller2, GamepadKeys.Button.LEFT_STICK_BUTTON);
        GamepadButton c2RightStickPress = new GamepadButton(controller2, GamepadKeys.Button.RIGHT_STICK_BUTTON);

        // Utility Buttons
        GamepadButton c2Back  = new GamepadButton(controller2, GamepadKeys.Button.BACK);
        GamepadButton c2Start = new GamepadButton(controller2, GamepadKeys.Button.START);


        //═════════════════════════════════════════════════════════════════════
        //  CONTROLLER 2 MAPPINGS
        //═════════════════════════════════════════════════════════════════════
        //
        // Add controller 2 mappings here as needed
    }
}