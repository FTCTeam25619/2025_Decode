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

    // currentAngle is always in SERVO degrees (2.0 – 249.0).
    // Start at servo midpoint → launch ~61°, clamped to 58° on first setLaunchAngle() call.
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

        //telemetry.addData("Hood: Servo Angle",  currentAngle);
        //telemetry.addData("Hood: Launch Angle", getCurrentLaunchAngle());
    }

    // ── Low-level: servo degrees (internal use only) ──────────────────────────

    /**
     * Sets the hood by raw servo position (2.0 – 249.0 degrees).
     * Use setLaunchAngle() instead — this is for internal conversions only.
     */
    public void setAngle(double degrees) {
        currentAngle = MathUtils.clamp(degrees,
                HoodConstants.Limits.hoodHardStopMin,   // 2.0
                HoodConstants.Limits.hoodHardStopMax);  // 249.0
    }

    // ── Mid-level: mechanical degrees (internal use only) ─────────────────────

    /**
     * Sets the hood by mechanical arm angle (15° – 34°).
     *   34° = hood up,   flattest → launch 48.25°
     *   15° = hood down, steepest → launch 74.3° (mechanical max)
     *
     * Servo = -13.0 * (mechDeg - 34) + 2.0
     * NOTE: Inverted — higher mech angle = lower servo value.
     */
    private void setMechanicalAngle(double mechanicalDegrees) {
        double clampedMech = MathUtils.clamp(
                mechanicalDegrees,
                HoodConstants.Limits.hoodMechMin,   // 15.0
                HoodConstants.Limits.hoodMechMax);  // 34.0

        double servoPos = HoodConstants.Limits.servoSlope
                * (clampedMech - HoodConstants.Limits.hoodMechMax)
                + HoodConstants.Limits.hoodHardStopMin;

        setAngle(servoPos);
    }

    /**
     * Back-calculates mechanical angle from current servo position.
     * Clamped to valid mechanical range.
     */
    private double getCurrentMechanicalAngle() {
        double mech = (currentAngle - HoodConstants.Limits.hoodHardStopMin)
                / HoodConstants.Limits.servoSlope
                + HoodConstants.Limits.hoodMechMax;

        return MathUtils.clamp(
                mech,
                HoodConstants.Limits.hoodMechMin,   // 15.0
                HoodConstants.Limits.hoodMechMax);  // 34.0
    }

    // ── High-level: launch angle — PRIMARY public interface ───────────────────

    /**
     * Sets the hood by launch angle (degrees above horizontal).
     *
     * SOFTWARE clamp: 48.25° – 58.0°
     * (True mechanical max is 74.3° but shots above 58° bounce out of goal)
     *
     * Interpolation uses the full mechanical range (48.25°–74.3°) so servo
     * positions are physically correct across the entire travel.
     *
     * This is the only method external callers should use.
     */
    public void setLaunchAngle(double launchDegrees) {
        // Software clamp — limits to 58° max even though mechanism goes to 74.3°
        double clampedLaunch = MathUtils.clamp(
                launchDegrees,
                HoodConstants.Limits.launchAngleMin,   // 48.25°
                HoodConstants.Limits.launchAngleMax);  // 58.0°

        // Interpolate across full mechanical range for correct servo positions
        // 48.25° → mech 34°,  74.3° → mech 15°
        double mechDegrees = HoodConstants.Limits.hoodMechMax
                + (clampedLaunch - HoodConstants.Limits.launchAngleMin)
                * (HoodConstants.Limits.hoodMechMin - HoodConstants.Limits.hoodMechMax)
                / (HoodConstants.Limits.launchAngleMechMax - HoodConstants.Limits.launchAngleMin);

        setMechanicalAngle(mechDegrees);
    }

    /**
     * Returns current launch angle (degrees above horizontal).
     *
     * Uses full mechanical range for the back-calculation, then clamps
     * to software range to prevent LUT out-of-bounds exceptions.
     */
    public double getCurrentLaunchAngle() {
        double mechDeg = getCurrentMechanicalAngle();

        // Back-calculate using full mechanical range
        double launchAngle = HoodConstants.Limits.launchAngleMin
                + (mechDeg - HoodConstants.Limits.hoodMechMax)
                * (HoodConstants.Limits.launchAngleMechMax - HoodConstants.Limits.launchAngleMin)
                / (HoodConstants.Limits.hoodMechMin - HoodConstants.Limits.hoodMechMax);

        // Clamp to software range — prevents LUT out-of-bounds
        return MathUtils.clamp(
                launchAngle,
                HoodConstants.Limits.launchAngleMin,   // 48.25°
                HoodConstants.Limits.launchAngleMax);  // 58.0°
    }

    // ── Geometry helpers ──────────────────────────────────────────────────────

    /**
     * Returns the launch height above floor (inches).
     * Variation across hood travel is small (0.59 in) so a fixed
     * average is used rather than interpolating.
     * Used by ShotController.launchHeightToGoal().
     */
    public double interpolateLaunchHeight() {
        return HoodConstants.Limits.launchHeightAvg;  // 15.39 in
    }

    // ── Status ────────────────────────────────────────────────────────────────

    public boolean atTargetAngle() {
        // FIXME: Replace with actual servo feedback thresholding when available
        return true;
    }
}
