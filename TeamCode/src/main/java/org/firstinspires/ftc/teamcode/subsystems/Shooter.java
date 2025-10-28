package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.Devices;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    MotorEx motorLeft;
    MotorEx motorRight;

    Telemetry telemetry;

    public enum ShooterState {
        Stopped,
        Shooting;
    }

    private ShooterState subsystemState = ShooterState.Stopped;

    private double flywheelTargetRPM = 0.0;
    private double priorTargetRPM = 0.0;
    private double  newVelocity = 0.0;
    private double motorPower = 0.0;

    private final PIDController flywheelVelocityPID;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        motorLeft = new MotorEx(hardwareMap, Devices.Shooter.leftMotor, Motor.GoBILDA.BARE);
        motorRight = new MotorEx(hardwareMap, Devices.Shooter.rightMotor, Motor.GoBILDA.BARE);

        flywheelVelocityPID = new PIDController(
                ShooterConstants.VelocityPID.kP,
                ShooterConstants.VelocityPID.kI,
                ShooterConstants.VelocityPID.kD,
                ShooterConstants.VelocityPID.kS,
                ShooterConstants.VelocityPID.kV
        );
        flywheelVelocityPID.reset();

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        super.periodic();

        switch(subsystemState) {
            case Stopped:
                motorPower = 0.0;
                flywheelVelocityPID.reset();
                break;
            case Shooting:
                if (flywheelTargetRPM != priorTargetRPM) {
                    flywheelVelocityPID.setSetPoint(this.flywheelTargetRPM);
                }
                newVelocity = flywheelVelocityPID.calculate(this.getFlywheelRPM());
                break;
        }

        this.runMotors(motorPower);

        this.telemetry.addData("Shooter: State", this.subsystemState.toString());
        this.telemetry.addData("Shooter: Target RPM", this.flywheelTargetRPM);
        this.telemetry.addData("Shooter: At Speed", this.atRPM());
        this.telemetry.addData("Shooter: Motor RPM", this.getFlywheelRPM());
        this.telemetry.addData("Shooter: Power", this.motorPower);
        this.telemetry.addData("Shooter: Calc Velocity", this.newVelocity);
    }

    public void setMotorPower(double power) {
        this.motorPower = MathUtils.clamp(power, -1.0, 1.0);
    }

    public void changeMotorPower(double powerChange) {
        this.setMotorPower(this.motorPower + powerChange);
    }

    public void setTargetRPM(double rpm) {
        this.flywheelTargetRPM = MathUtils.clamp(rpm, 0.0, 5800.0);
    }

    public double getFlywheelRPM() {
        return this.motorLeft.getVelocity() *
                Constants.ConversionFactors.SECONDS_PER_MINUTE /
                this.motorLeft.getCPR();
    }

    public boolean atRPM() {
        return Math.abs(getFlywheelRPM() - this.flywheelTargetRPM) <= ShooterConstants.flywheelRPMTolerance;
    }

    public void setState(ShooterState newState) {
        this.subsystemState = newState;
    }

    private void runMotors(double power) {
        motorLeft.set(power);
        motorRight.set(-power);
    }
}
