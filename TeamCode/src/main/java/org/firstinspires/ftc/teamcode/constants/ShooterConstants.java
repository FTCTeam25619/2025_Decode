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
        public static double idleRPM = 2800.0;
        public static double atGoalShot = 1850.0;
        public static double nearZoneMidShot = 2275.0;
        public static double nearZoneEndShot = 2400.0;
        public static double farWallShot = 2960.0;
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
