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
        // ── Servo units ───────────────────────────────────────────────────────
        // NOTE: Servo and mechanical angles are INVERTED.
        // A lower servo position produces a higher mechanical (and launch) angle.
        //
        // Servo 28.5  → Mech 34° → Launch 56.0° (hood up, flattest, far shots)
        // Servo 275.0 → Mech 15° → Launch 72.8° (hood down, steepest, close shots)

        public static double hoodHardStopMin = 2.0;   // servo pos at mech MAX (34°)
        public static double hoodHardStopMax = 249.0;  // servo pos at mech MIN (15°)

        // ── Mechanical degrees ────────────────────────────────────────────────
        public static double hoodMechMin = 15.0;  // hood down — steepest — close shots
        public static double hoodMechMax = 34.0;  // hood up   — flattest — far shots

        // Slope for mechanical → servo conversion
        // Servo Position = servoSlope * (mechDeg - hoodMechMax) + hoodHardStopMin
        // = -13.0 * (mechDeg - 34) + 4.0
        public static double servoSlope = -10.04;

        // ── Launch angles (measured from horizontal) ──────────────────────────
        // These are the actual ball exit angles measured physically on the robot.
        // Launch angle increases (steeper) as mechanical angle decreases.
        //
        // Mech 34° (hood up,   far shots)   → Launch 50.3° above horizontal
        // Mech 15° (hood down, close shots) → Launch 74.9° above horizontal

        public static double launchAngleAtMechMax = 50.3;  // degrees — flattest launch
        public static double launchAngleAtMechMin = 74.9;  // degrees — steepest launch

        // ── Launch heights (measured physically on the robot) ─────────────────
        // Height of the ball exit point above the floor, varies with hood position.
        //
        // Mech 34° (hood up)   → ball exits at 15.85 inches above floor
        // Mech 15° (hood down) → ball exits at 14.56 inches above floor

        public static double launchHtAtMechMax = 15.85;  // inches — hood up
        public static double launchHtAtMechMin = 14.56;  // inches — hood down
    }

    @Configurable
    public static class Control {
        public static double incrementSize = 10.0;
    }

    @Configurable
    public static class Targets {
        // Preset shot targets expressed in SERVO degrees.
        // Use robot.hood.setMechanicalAngle() or setLaunchAngle() for
        // physics-solver driven shots instead of these raw servo values.
        public static double atGoalShot     = Limits.hoodHardStopMax;  // servo 275 → mech 15° → launch 72.8°
        public static double nearZoneMidShot = Limits.hoodHardStopMin; // servo 28.5 → mech 34° → launch 56°
        public static double nearZoneEndShot = Limits.hoodHardStopMin;
        public static double farWallShot     = Limits.hoodHardStopMin;
    }
}