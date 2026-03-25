package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

public class ShooterFeedConstants {
    @Configurable
    public static class Targets {
        public static double defaultFeedPower = 1.0;
        public static double intakeFeedPower = 0.7;
        public static double atGoalShot = 1.0;
        public static double nearZoneMidShot = 1.0;
        public static double nearZoneEndShot = 1.0;
        public static double farWallShot = 1.0;
    }
}
