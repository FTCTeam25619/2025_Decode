package org.firstinspires.ftc.teamcode.lib.orchestration;

public enum ShotCycleState {
    /**
     * State: OFF
     * Used in Auto
     */
    OFF,
    /**
     * State: IDLE
     * Awaiting next shot cycle
     * Prior states: none or any (default state)
     * Effects: none
     * Trigger: Starting point and automatic return state when shooting completes
     */
    IDLE,
    /**
     * State: INTAKE
     * Intaking game pieces
     * Prior states: IDLE
     * Effects:
     *   - intake motors on
     *   - feed stop closed
     * Trigger: Driver action - LT button held (auto-intake)
     */
    INTAKE,
    /**
     * State: STAGED
     * Shot(s) staged in feed behind feed stop
     * Prior states: INTAKE
     * Effects:
     *   - shooter flywheels at idle speed
     *   - feed stop remains closed
     * Trigger: Sensors see at least one game piece
     */
    STAGED, // FIXME: State or trigger condition?
    /**
     * State: PREPARE_SHOT
     * Setting up for shooting, awaiting tolerances to be met
     * Prior states: STAGED
     * Effects:
     *   - shooter flywheels to target RPM for shot
     *   - hood to target angle for shot
     *   - feed stop opens
     * Trigger: Driver action - RT button held (auto-shoot)
     */
    PREPARE_SHOT,
    /**
     * State: READY_TO_SHOOT
     * Shooting thresholds have been met and shooting can begin
     * Prior states: PREPARE_SHOT
     * Effects: none
     * Trigger: shooter flywheel RPM within tolerance, hood position within tolerance
     */
    READY_TO_SHOOT, // FIXME: State or trigger condition?
    /**
     * State: SHOOTING
     * Feeding game pieces through the shooter flywheels
     * Prior states: READY_TO_SHOOT (when auto-shooting) or any (in preset mode)
     * Effects:
     *   - feed wheels on
     * Triggers:
     *   - Driver action: RB button pressed (force shot)
     *   - Auto-shooting: not in shot preset mode
     */
    // Shooting: Feed wheels on
    SHOOTING;
}
