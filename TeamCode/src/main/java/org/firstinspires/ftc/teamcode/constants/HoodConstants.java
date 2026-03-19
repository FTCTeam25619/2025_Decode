package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

public class HoodConstants {

    @Configurable
    public static class DeviceConfig {
        // The angle in degrees of the servo when set to 0.0
        public static double hoodServoAngleMin = 0.0;
        // The angle in degrees of the servo when set to 1.0
        public static double hoodServoAngleMax = 360.0;
    }

    @Configurable
    public static class Limits {
        // ── Launch angles — PRIMARY public interface ──────────────────────────
        // Actual ball exit angles measured from horizontal.
        // Use setLaunchAngle() / getCurrentLaunchAngle() everywhere outside Hood.java.
        //
        // True mechanical range: 48.25° to 74.3°
        // Software clamp:        48.25° to 58.0°
        //
        // Shots steeper than 58° produce excessive arc and bounce out of the goal.
        // The servo hard stops reflect the TRUE mechanical range (48.25°–74.3°).
        // setLaunchAngle() clamps to launchAngleMax (58°) before commanding the servo.

        public static double launchAngleMin     = 48.25;  // flattest  — hard stop
        public static double launchAngleMax     = 58.0;   // steepest  — SOFTWARE clamp
        public static double launchAngleMechMax = 74.3;   // true mechanical limit — not used by solver

        // ── Launch height ─────────────────────────────────────────────────────
        // Ball exit height above floor. Variation across hood travel is small
        // so a fixed average is used in the physics solver.
        //
        // Hood at 48.25° → 15.69 in above floor
        // Hood at 58.0°  → 15.10 in above floor
        // Average: 15.39 in

        public static double launchHeightAvg = 15.39;  // inches — fixed average

        // ── Servo units (internal use only — do not use outside Hood.java) ────
        // Hard stops reflect the TRUE mechanical range (48.25° to 74.3°).
        // Servo 2.0   → Launch 48.25° (flattest)
        // Servo 249.0 → Launch 74.3°  (true mechanical max)
        // Servo ~94.4 → Launch 58.0°  (software clamp position)

        public static double hoodHardStopMin = 2.0;    // servo pos at launch MIN (48.25°)
        public static double hoodHardStopMax = 249.0;  // servo pos at launch MECH MAX (74.3°)

        // ── Mechanical degrees (internal use only — do not use outside Hood.java) ──
        // mech 34° corresponds to launch 48.25° (flattest)
        // mech 15° corresponds to launch 74.3°  (true mechanical max)

        public static double hoodMechMin = 15.0;   // hood down — steepest (true mech max)
        public static double hoodMechMax = 34.0;   // hood up   — flattest

        // Servo = servoSlope * (mechDeg - hoodMechMax) + hoodHardStopMin
        //       = -13.0 * (mechDeg - 34) + 2.0
        public static double servoSlope = -13.0;
    }

    @Configurable
    public static class Control {
        public static double incrementSize = 1.0;  // degrees — launch angle increment
    }

    @Configurable
    public static class Targets {
        // Preset shot targets in LAUNCH ANGLE degrees.
        // Pass to robot.hood.setLaunchAngle().
        public static double atGoalShot      = Limits.launchAngleMax;  // 58.0° — close, steep
        public static double nearZoneMidShot = 53.0;                   // mid range
        public static double nearZoneEndShot = 53.0;
        public static double farWallShot     = Limits.launchAngleMin;  // 48.25° — far, flat
    }
}