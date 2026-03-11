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
import org.firstinspires.ftc.teamcode.constants.KickerConstants;


public class Kicker extends SubsystemBase {

    private final MotorEx motor;
    private final Telemetry telemetry;

    public enum KickerState {
        Stopped,
        ManualSpin,
        Spinning
    }

    private KickerState subsystemState = KickerState.Spinning;

    private double targetRPM  = KickerConstants.Targets.idleRPM;
    private double motorPower = 0.0;


    public Kicker(HardwareMap hardwareMap, Telemetry telemetry) {
        motor = new MotorEx(hardwareMap, Devices.Kicker.motor, Motor.GoBILDA.BARE);
        motor.setInverted(KickerConstants.motorInverted);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor.motorEx.setVelocityPIDFCoefficients(
                KickerConstants.VelocityPID.kP,
                KickerConstants.VelocityPID.kI,
                KickerConstants.VelocityPID.kD,
                KickerConstants.VelocityPID.kF
        );

        PIDFCoefficients veloPIDCoeff =
                motor.motorEx.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        this.telemetry = telemetry;
        if (Constants.DebugModes.Telemetry.DEBUG_KICKER) {
            telemetry.addData("Kicker Velo PID P", veloPIDCoeff.p);
            telemetry.addData("Kicker Velo PID I", veloPIDCoeff.i);
            telemetry.addData("Kicker Velo PID D", veloPIDCoeff.d);
            telemetry.addData("Kicker Velo PID F", veloPIDCoeff.f);
        }
    }


    @Override
    public void periodic() {
        super.periodic();

        switch (subsystemState) {
            case Stopped:
                motorPower = 0.0;
                runMotor(motorPower);
                break;
            case ManualSpin:
                runMotor(motorPower);
                break;
            case Spinning:
                setMotorVelocity(targetRPM);
                break;
        }

        telemetry.addData("Kicker At Speed",    this.atRPM());
        telemetry.addData("Kicker Target RPM",  this.targetRPM);
        telemetry.addData("Kicker Actual RPM",  this.getKickerRPM());

        if (Constants.DebugModes.Telemetry.DEBUG_KICKER) {
            telemetry.addData("Kicker RPM Thr Above", targetRPM + KickerConstants.rpmTolerance);
            telemetry.addData("Kicker RPM Thr Below", targetRPM - KickerConstants.rpmTolerance);
            telemetry.addData("Kicker State",          subsystemState.toString());
            telemetry.addData("Kicker Power",          motorPower);
            telemetry.addData("Kicker Motor Power",    motor.motorEx.getPower());
        }
    }

    public void setMotorPower(double power) {
        this.motorPower = MathUtils.clamp(power, -1.0, 1.0);
    }

    public void changeMotorPower(double delta) {
        setMotorPower(motorPower + delta);
    }

    public void setTargetRPM(double rpm) {
        this.targetRPM = MathUtils.clamp(
                rpm,
                KickerConstants.minRPM,
                KickerConstants.maxRPM
        );
    }

    public void changeTargetRPM(double delta) {
        setTargetRPM(targetRPM + delta);
    }


    public void setTargetRPMFromFlywheel(double flywheelRPM) {
        setTargetRPM(flywheelRPM * KickerConstants.flywheelRPMMultiplier);
    }

    public double getKickerRPM() {
        return motor.getVelocity()
                * KickerConstants.gearRatio
                * Constants.ConversionFactors.SECONDS_PER_MINUTE
                / motor.getCPR();
    }

    public boolean atRPM() {
        return Math.abs(getKickerRPM() - targetRPM) <= KickerConstants.rpmTolerance;
    }

    public void setState(KickerState newState) {
        this.subsystemState = newState;
    }

    public KickerState getState() {
        return subsystemState;
    }


    private void runMotor(double power) {
        motor.set(power);
    }

    private void setMotorVelocity(double rpm) {
        double velocityCPS = rpm * motor.getCPR()
                / (Constants.ConversionFactors.SECONDS_PER_MINUTE * KickerConstants.gearRatio);
        motor.motorEx.setVelocity(velocityCPS);
    }
}