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
        Stopped;
    }

    private IntakeState subsystemState = IntakeState.Stopped;
    private double motorPower = 0.0;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        intakeMotor = new MotorEx(hardwareMap, Devices.Intake.intakeMotor, Motor.GoBILDA.RPM_1150);
        intakeMotor.setInverted(true);
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        switch (subsystemState) {
            case Stopped:
                motorPower = 0.0;
                break;
            case Dejam:
                motorPower = IntakeConstants.dejamPower;
                break;
            case Intake:
                motorPower = IntakeConstants.intakePower;
                break;
            case Manual:
                break;
        }
        intakeMotor.set(motorPower);

        telemetry.addData("Intake: Motor Power", motorPower);
        telemetry.addData("Intake: State", subsystemState.toString());
    }

    public void setManualMotorPower(double newPower) {
        this.motorPower = MathUtils.clamp(motorPower, -1.0, 1.0);
    }

    public void setState(IntakeState newState) {
        this.subsystemState = newState;
    }
}
