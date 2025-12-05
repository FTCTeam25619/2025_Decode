package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;
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
        ManualSpin,
        Shooting;
    }

    private ShooterState subsystemState = ShooterState.Stopped;

    private double flywheelTargetRPM = 2500.0;
    private double motorPower = 0.0;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        motorLeft = new MotorEx(hardwareMap, Devices.Shooter.leftMotor, Motor.GoBILDA.BARE);
        motorRight = new MotorEx(hardwareMap, Devices.Shooter.rightMotor, Motor.GoBILDA.BARE);
        motorLeft.setInverted(false);
        motorRight.setInverted(true);
        motorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motorLeft.motorEx.setVelocityPIDFCoefficients(
            ShooterConstants.VelocityPID.kP,
            ShooterConstants.VelocityPID.kI,
            ShooterConstants.VelocityPID.kD,
            ShooterConstants.VelocityPID.kF
        );
        motorRight.motorEx.setVelocityPIDFCoefficients(
            ShooterConstants.VelocityPID.kP,
            ShooterConstants.VelocityPID.kI,
            ShooterConstants.VelocityPID.kD,
            ShooterConstants.VelocityPID.kF
        );

        PIDFCoefficients veloPIDCoeff = motorLeft.motorEx.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        this.telemetry = telemetry;
        if (Constants.DebugModes.Telemetry.DEBUG_SHOOTER) {
            telemetry.addData("Velo PID P", veloPIDCoeff.p);
            telemetry.addData("Velo PID I", veloPIDCoeff.i);
            telemetry.addData("Velo PID D", veloPIDCoeff.d);
            telemetry.addData("Velo PID F", veloPIDCoeff.f);
        }
    }

    @Override
    public void periodic() {
        super.periodic();

        switch(subsystemState) {
            case Stopped:
                motorPower = 0.0;
                this.runMotors(motorPower);
                break;
            case ManualSpin:
                this.runMotors(motorPower);
                break;
            case Shooting:
                this.setMotorVelocity(this.flywheelTargetRPM);
                break;
        }


        this.telemetry.addData("Shooter: Target RPM", this.flywheelTargetRPM);
        this.telemetry.addData("Shooter: At Speed", this.atRPM());
        if (Constants.DebugModes.Telemetry.DEBUG_SHOOTER) {
            this.telemetry.addData("Shooter: State", this.subsystemState.toString());
            this.telemetry.addData("Shooter: Power", this.motorPower);
            this.telemetry.addData("Shooter: Motor RPM", this.getFlywheelRPM());
            this.telemetry.addData("Shooter: L Motor Power", this.motorLeft.motorEx.getPower());
            this.telemetry.addData("Shooter: R Motor Power", this.motorRight.motorEx.getPower());
        }
    }

    // No motor belted -- 6100 RPM both
    // R motor belted -- 5800 L, 3500 R
    // L motor belted -- 3500 L, 5850 R
    // both belted -- 3850 L, 3850 R

    public void setMotorPower(double power) {
        this.motorPower = MathUtils.clamp(power, -1.0, 1.0);
    }

    public void changeMotorPower(double powerChange) {
        this.setMotorPower(this.motorPower + powerChange);
    }

    public void setTargetRPM(double rpm) {
        this.flywheelTargetRPM = MathUtils.clamp(rpm,
                ShooterConstants.minFlywheelRPM, ShooterConstants.maxFlywheelRPM);
    }

    public void changeTargetRPM(double rpmChange) {
        this.setTargetRPM(this.flywheelTargetRPM + rpmChange);
    }

    public double getFlywheelRPM() {
        return this.motorLeft.getVelocity() * ShooterConstants.gearRatio *
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
        motorRight.set(power);
    }

    private void setMotorVelocity(double rpm) {
        double velocityCPS = rpm * motorLeft.getCPR() /
                (Constants.ConversionFactors.SECONDS_PER_MINUTE * ShooterConstants.gearRatio);
        motorLeft.motorEx.setVelocity(velocityCPS);
        motorRight.motorEx.setVelocity(velocityCPS);
    }
}
