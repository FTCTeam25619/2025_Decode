package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Devices;

public class Exploration extends SubsystemBase {
    private final Motor motor1;
    private final AnalogInput absoluteEncoder;
    private final Telemetry telemetry;

    public Exploration(HardwareMap hardwareMap, Telemetry telemetry) {
        motor1 = new Motor(hardwareMap, Devices.Exploration.motor1);
        absoluteEncoder = hardwareMap.get(AnalogInput.class, Devices.Exploration.rotarySensor);

        this.telemetry = telemetry;
    }

    public void moveMotor(double power) {
        double clamped = Math.max(-1.0, Math.min(1.0, power));
        telemetry.addData("Clamped Power", clamped);
        motor1.set(clamped);
    }

    public double readRotarySensor(){
        double reading = absoluteEncoder.getVoltage();
        return reading;

    }

    public static class MoveMotor extends CommandBase {
        private final GamepadEx mController1;
        private final Exploration mSubsystem;
        private final Telemetry mTelemetry;

        public MoveMotor(Exploration subsystem, GamepadEx controller1, Telemetry robotTelemetry) {
            mSubsystem = subsystem;
            mController1 = controller1;
            mTelemetry = robotTelemetry;

            addRequirements(mSubsystem);
        }

        @Override
        public void initialize() {
            super.initialize();
        }

        @Override
        public void execute() {
            double power = mController1.getLeftX();
            mTelemetry.addData("Requested Power", power);
            mTelemetry.addData("Voltage", mSubsystem.readRotarySensor());
            mSubsystem.moveMotor(power);
        }

        @Override
        public void end(boolean interrupted) {
            mSubsystem.moveMotor(0.0);
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }
}
