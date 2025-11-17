package org.firstinspires.ftc.teamcode.lib.servos;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.util.MathUtils;

public class ContinuousServo {
    private final CRServo servo;

    public enum ContinuousServoDirection {
        REVERSE(-1.0),
        STOP(0.0),
        FORWARD(1.0);

        public final double power;

        ContinuousServoDirection(double pwr) {
            this.power = pwr;
        }
    }

    public ContinuousServo(HardwareMap hw, String servoName) {
        servo = hw.get(CRServo.class, servoName);
    }

    /*
     * Move the servo in the indicated direction at full power.
     */
    public void move(ContinuousServoDirection direction) {
        servo.setPower(direction.power);
    }

    /*
     * Stop the servo.
     */
    public void stop() {
        servo.setPower(ContinuousServoDirection.STOP.power);
    }

    /*
     * Move the servo in the positive-power direction at full power.
     */
    public void forward() {
        servo.setPower(ContinuousServoDirection.FORWARD.power);
    }

    /*
     * Move the servo in the positive-power direction at the desired power.
     */
    public void forward(double power) { servo.setPower(MathUtils.clamp(power, 0.0, 1.0)); }

    /*
     * Move the servo in the negative-power direction at full power.
     */
    public void reverse() {
        servo.setPower(ContinuousServoDirection.REVERSE.power);
    }

    /*
     * Move the servo in the negative-power direction at the desired power.
     */
    public void reverse(double power) {
        servo.setPower(MathUtils.clamp(-power, -1.0, 0.0));
    }

    /*
     * Set whether the direction of the servo is inverted or not.
     */
    public void setInverted(boolean isInverted) {
        servo.setDirection(isInverted ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    /*
     * Get the inverted setting of the servo.
     */
    public boolean getInverted() {
        return DcMotorSimple.Direction.REVERSE == servo.getDirection();
    }

    /*
     * Get the current power setting of the servo.
     */
    public double getPower() {
        return servo.getPower();
    }

    /*
     * Get the current movement direction of the servo.
     */
    public ContinuousServoDirection getCurrentMovement() {
        double currentPower = servo.getPower();
        if (currentPower < ContinuousServoDirection.STOP.power) { return ContinuousServoDirection.REVERSE; }
        if (currentPower > ContinuousServoDirection.STOP.power) { return ContinuousServoDirection.FORWARD; }
        return ContinuousServoDirection.STOP;
    }

    /*
     * Disable the servo.
     */
    public void disable() {
        servo.close();
    }

    /*
     * Get device identifier and port information for the servo.
     */
    public String getDeviceType() {
        String port = Integer.toString(servo.getPortNumber());
        String controller = servo.getController().toString();
        return "ContinuousServo: " + port + "; " + controller;
    }
}