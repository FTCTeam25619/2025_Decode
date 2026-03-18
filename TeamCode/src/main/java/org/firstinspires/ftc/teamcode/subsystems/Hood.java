package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Devices;
import org.firstinspires.ftc.teamcode.constants.HoodConstants;

public class Hood extends SubsystemBase {
    private final ServoEx hoodServo;
    private final Telemetry telemetry;

    // currentAngle is always in SERVO degrees (28.5 – 275.0).
    // Start at servo midpoint which corresponds to mech ~24.75° / launch ~64.4°.
    private double currentAngle = (HoodConstants.Limits.hoodHardStopMin +
            HoodConstants.Limits.hoodHardStopMax) / 2.0;

    public Hood(HardwareMap hardwareMap, Telemetry telemetry) {
        hoodServo = new ServoEx(hardwareMap, Devices.Hood.hoodServo,
                HoodConstants.DeviceConfig.hoodServoAngleMin,
                HoodConstants.DeviceConfig.hoodServoAngleMax);
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        hoodServo.set(currentAngle);

        //telemetry.addData("Hood: Servo Angle", currentAngle);
        //telemetry.addData("Hood: Mech Angle",  getCurrentMechanicalAngle());
        //telemetry.addData("Hood: Launch Angle", getCurrentLaunchAngle());
    }

    // ── Low-level: servo degrees ──────────────────────────────────────────────

    /**
     * Sets the hood directly by servo position (28.5 – 275.0 degrees).
     * Prefer setMechanicalAngle() or setLaunchAngle() over calling this directly.
     */
    public void setAngle(double degrees) {
        currentAngle = MathUtils.clamp(degrees,
                HoodConstants.Limits.hoodHardStopMin,
                HoodConstants.Limits.hoodHardStopMax);
    }

    /**
     * Moves the hood by a delta in servo degrees.
     */
    public void moveBy(double degrees) {
        setAngle(currentAngle + degrees);
    }

    // ── Mid-level: mechanical degrees (15° – 34°) ─────────────────────────────

    /**
     * Sets the hood by mechanical angle (15° – 34°).
     *
     * Mechanical angle is the physical arm angle:
     *   15° = hood down, steepest, close shots
     *   34° = hood up,   flattest, far shots
     *
     * Conversion:
     *   Servo = servoSlope * (mechDeg - hoodMechMax) + hoodHardStopMin
     *         = -12.97 * (mechDeg - 34) + 28.5
     *
     * NOTE: Relationship is inverted — higher mech angle = lower servo value.
     */
    public void setMechanicalAngle(double mechanicalDegrees) {
        double clampedMech = MathUtils.clamp(
                mechanicalDegrees,
                HoodConstants.Limits.hoodMechMin,   // 15.0
                HoodConstants.Limits.hoodMechMax);  // 34.0

        double servoPos = HoodConstants.Limits.servoSlope
                * (clampedMech - HoodConstants.Limits.hoodMechMax)
                + HoodConstants.Limits.hoodHardStopMin;  // 28.5

        setAngle(servoPos);
    }

    /**
     * Back-calculates the current mechanical angle from the servo position.
     * Inverse of the setMechanicalAngle() conversion.
     *
     * mechDeg = (servoPos - hoodHardStopMin) / servoSlope + hoodMechMax
     */
    public double getCurrentMechanicalAngle() {
        return (currentAngle - HoodConstants.Limits.hoodHardStopMin)
                / HoodConstants.Limits.servoSlope
                + HoodConstants.Limits.hoodMechMax;
    }

    // ── High-level: launch angle (56.0° – 72.8° from horizontal) ─────────────

    /**
     * Sets the hood by launch angle (degrees above horizontal).
     *
     * Launch angle is the actual ball exit angle measured from horizontal:
     *   56.0° = flattest launch — mech 34° — far shots
     *   72.8° = steepest launch — mech 15° — close shots
     *
     * This is the method the physics solver calls.
     *
     * Conversion: launch → mech → servo (two-step via setMechanicalAngle)
     */
    public void setLaunchAngle(double launchDegrees) {
        double clampedLaunch = MathUtils.clamp(
                launchDegrees,
                HoodConstants.Limits.launchAngleAtMechMax,  // 56.0° — flattest
                HoodConstants.Limits.launchAngleAtMechMin); // 72.8° — steepest

        // Linear interpolation: launch → mechanical
        // launch 56.0° → mech 34°,  launch 72.8° → mech 15°
        double mechDegrees = HoodConstants.Limits.hoodMechMax
                + (clampedLaunch - HoodConstants.Limits.launchAngleAtMechMax)
                * (HoodConstants.Limits.hoodMechMin - HoodConstants.Limits.hoodMechMax)
                / (HoodConstants.Limits.launchAngleAtMechMin - HoodConstants.Limits.launchAngleAtMechMax);

        setMechanicalAngle(mechDegrees);
    }

    /**
     * Returns the current launch angle in degrees above horizontal,
     * derived from the current servo position.
     */
    public double getCurrentLaunchAngle() {
        double mechDeg = getCurrentMechanicalAngle();
        double launchAngle = HoodConstants.Limits.launchAngleAtMechMax
                + (mechDeg - HoodConstants.Limits.hoodMechMax)
                * (HoodConstants.Limits.launchAngleAtMechMin - HoodConstants.Limits.launchAngleAtMechMax)
                / (HoodConstants.Limits.hoodMechMin - HoodConstants.Limits.hoodMechMax);

        // Clamp to valid range — prevents LUT out-of-bounds if servo
        // position is outside expected range at startup or during transition
        return MathUtils.clamp(
                launchAngle,
                HoodConstants.Limits.launchAngleAtMechMax,  // 50.0°
                HoodConstants.Limits.launchAngleAtMechMin); // 72.8°
    }

    // ── Geometry helpers ──────────────────────────────────────────────────────

    /**
     * Returns the interpolated launch height (inches above floor) based on
     * the current mechanical angle.
     *
     * Measured values:
     *   Mech 34° → 15.85 in above floor
     *   Mech 15° → 14.56 in above floor
     *
     * Used by ShotController.launchHeightToGoal() in the physics solver.
     */
    public double interpolateLaunchHeight() {
        double mechDeg = getCurrentMechanicalAngle();
        return HoodConstants.Limits.launchHtAtMechMin
                + (mechDeg - HoodConstants.Limits.hoodMechMin)
                * (HoodConstants.Limits.launchHtAtMechMax - HoodConstants.Limits.launchHtAtMechMin)
                / (HoodConstants.Limits.hoodMechMax - HoodConstants.Limits.hoodMechMin);
    }

    // ── Status ────────────────────────────────────────────────────────────────

    public boolean atTargetAngle() {
        // FIXME: Replace with actual servo feedback thresholding when available
        return true;
    }
}
