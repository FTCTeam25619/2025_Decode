package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Devices;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final MotorEx intakeMotor;
    private final Telemetry telemetry;

    public enum IntakeState {
        Intake,
        Dejam,
        Manual,
        Stopped,
        Stalled;  // New state for stall condition
    }

    private IntakeState subsystemState = IntakeState.Stopped;
    private IntakeState previousState = IntakeState.Stopped;  // Track previous state
    private double motorPower = 0.0;

    // Stall detection variables
    private int lastEncoderPosition = 0;
    private int stallCounter = 0;
    private boolean isStalled = false;
    private static final int STALL_THRESHOLD_TICKS = 5;  // Encoder change threshold
    private static final int STALL_CYCLES = 3;  // Number of cycles before declaring stall
    private static final double MIN_POWER_FOR_STALL_CHECK = 0.2;  // Only check stall when motor should be running

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        intakeMotor = new MotorEx(hardwareMap, Devices.Intake.intakeMotor, Motor.GoBILDA.RPM_1150);
        intakeMotor.setInverted(true);
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intakeMotor.resetEncoder();  // Reset encoder at initialization

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        // Check for stall condition before setting motor power
        checkForStall();

        switch (subsystemState) {
            case Stopped:
                motorPower = 0.0;
                resetStallDetection();
                break;
            case Dejam:
                motorPower = IntakeConstants.dejamPower;
                break;
            case Intake:
                motorPower = IntakeConstants.intakePower;
                break;
            case Manual:
                // Power is set externally via setManualMotorPower
                break;
            case Stalled:
                motorPower = 0.0;
                break;
        }

        intakeMotor.set(motorPower);

       // telemetry.addData("Intake: Motor Power", motorPower);
       // telemetry.addData("Intake: State", subsystemState.toString());
       // telemetry.addData("Intake: Encoder Position", intakeMotor.getCurrentPosition());
       // telemetry.addData("Intake: Stalled", isStalled);
       // telemetry.addData("Intake: Stall Counter", stallCounter);
    }

    private void checkForStall() {
        // Only check for stall if motor should be running
        if (Math.abs(motorPower) < MIN_POWER_FOR_STALL_CHECK) {
            return;
        }

        // Don't check stall if already in stalled or stopped state
        if (subsystemState == IntakeState.Stalled || subsystemState == IntakeState.Stopped) {
            return;
        }

        int currentPosition = intakeMotor.getCurrentPosition();
        int positionChange = Math.abs(currentPosition - lastEncoderPosition);

        // If encoder hasn't moved much, increment stall counter
        if (positionChange < STALL_THRESHOLD_TICKS) {
            stallCounter++;

            // If stalled for multiple cycles, trigger stall state
            if (stallCounter >= STALL_CYCLES && !isStalled) {
                isStalled = true;
                previousState = subsystemState;  // Save current state
                subsystemState = IntakeState.Stalled;
            }
        } else {
            // Motor is moving, reset stall counter
            stallCounter = 0;
            isStalled = false;
        }

        lastEncoderPosition = currentPosition;
    }

    private void resetStallDetection() {
        stallCounter = 0;
        isStalled = false;
        lastEncoderPosition = intakeMotor.getCurrentPosition();
    }

    public void setManualMotorPower(double newPower) {
        this.motorPower = MathUtils.clamp(newPower, -1.0, 1.0);  // Fixed: was clamping motorPower instead of newPower
    }

    public void setState(IntakeState newState) {
        // If transitioning from Stalled to Stopped, reset stall detection
        if (subsystemState == IntakeState.Stalled && newState == IntakeState.Stopped) {
            resetStallDetection();
        }

        // If trying to set intake/dejam/manual while stalled, ignore until driver releases trigger
        if (isStalled && (newState == IntakeState.Intake || newState == IntakeState.Dejam || newState == IntakeState.Manual)) {
            return;  // Don't allow state change until stall is cleared
        }

        this.subsystemState = newState;
    }

    public boolean isStalled() {
        return isStalled;
    }

    public IntakeState getState() {
        return subsystemState;
    }
}