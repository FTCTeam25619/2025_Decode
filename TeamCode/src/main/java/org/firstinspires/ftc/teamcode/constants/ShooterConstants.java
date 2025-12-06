package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

public class ShooterConstants {
    public static final double flywheelRPMTolerance = 100.0;
    public static final double minFlywheelRPM = 0.0;
    public static final double maxFlywheelRPM = 8000.0;
    public static final double gearRatio = 1.0;

    public static class VelocityPID {
        public static final double kP = 600.0; // 300.0;
        public static final double kI = 0.0;
        public static final double kD = 5.0;
        public static final double kF = 21.0;
    }

    @Config
    @Configurable
    public static class Targets {
        public static double idleRPM = 2800.0;
        public static double atGoalShot = 1800.0;
        public static double nearZoneMidShot = 2100.0;
        public static double nearZoneEndShot = 2400.0;
        public static double farWallShot = 3200.0;
    }
}

/**
 * R1: 2528.5 -> 2185.7, 2228.5, 2271.4
 * R3: 2528.5 -> 2228.5, 2185.7, 2228.5
 * 300 RPM +/- 50 loss with ball passing through
 */

/*
 * Interp Table
 *   Far zone: 3200 RPM, 0.2 Feed, Hood all the way up
 *   Near zone far end: 2400 RPM, 0.32 Feed, Hood halfway
 *   Near zone layup: 1800 RPM, 0.34 Feed, Hood all the way down
 */
