package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterConstants {
    public static double flywheelRPMTolerance = 160.0;
    public static double minFlywheelRPM = 0.0;
    public static double maxFlywheelRPM = 8000.0;
    public static double gearRatio = 1.0;

    @Configurable
    public static class VelocityPID {
        public static double kP = 630.0;
        public static double kI = 0.0;
        public static double kD = 5.0;
        public static double kF = 21.0;
    }

    @Configurable
    public static class Targets {
        public static double idleRPM = 2350;
        public static double atGoalShot = 1850.0;
        public static double nearZoneMidShot = 2275.0;
        public static double nearZoneEndShot = 2550.0;
        public static double farWallShot = 2350;
    }

    /**
     * Physics solver constants for the autoTargetShot() implementation.
     *
     * Overview:
     *   - RPM controls distance; hood angle compensates for RPM drop between shots.
     *   - Two baseline RPMs are commanded (one per zone) — always higher than the
     *     minimum needed so the hood has downward headroom to absorb drops.
     *   - Actual RPM is read per-shot and fed into the physics solver to compute
     *     the compensating hood angle.
     *   - A v0 correction LUT absorbs any systematic deviation of the empirical
     *     curve from real-world performance across the distance range.
     *
     * Calibration:
     *   - v0Slope and v0Intercept come from the ShooterCalibration spreadsheet
     *     after running the calibration OpMode and fitting the RPM→v0 curve.
     *   - v0CorrectionLUT values start at 1.0 and are tuned empirically after
     *     first real goal shots — adjust per distance point until shots land true.
     *   - baselineRPM_Far and baselineRPM_Close should be set ~200 RPM above the
     *     highest RPM needed at each zone (confirmed from Solver Preview sheet).
     */
    @Configurable
    public static class Solver {
        // ── Physics constants ─────────────────────────────────────────────────
        public static final double G = 386.1;  // gravity in/s²

        // Goal center height = goal lip (38.75 in) + ball radius (2.5 in)
        public static final double goalCenterHeight = 41.25;  // inches above floor

        // ── Distance bounds ───────────────────────────────────────────────────
        public static double minDistanceIn = 35.0;   // closest shot (inches)
        public static double maxDistanceIn = 160.0;  // farthest shot (inches)


        // ── Empirical v0 curve ────────────────────────────────────────────────
        // Linear fit from calibration: v0 (in/s) = v0Slope * RPM + v0Intercept
        // Both values come from the Curve Fit sheet in ShooterCalibration.xlsx.
        // Leave at 0.0 until calibration is complete.
        public static double v0Slope     = 0.0;  // in/s per RPM — from calibration
        public static double v0Intercept = 0.0;  // in/s          — from calibration
    }
}
