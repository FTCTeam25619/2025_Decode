package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

public class ShooterFeedConstants {
    @Config
    @Configurable
    public static class Targets {
        public static double defaultFeedPower = 1.0;
        public static double atGoalShot = 0.34;
        public static double nearZoneMidShot = 0.32;
        public static double nearZoneEndShot = 0.32;
        public static double farWallShot = 0.2;
    }
}
