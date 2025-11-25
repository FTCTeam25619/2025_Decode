package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

public class HoodConstants {
    @Config
    @Configurable
    public static class DeviceConfig {
        // The angle in degrees of the Hood when the servo is set to 0.0
        public static double hoodServoAngleMin = 0.0;
        // The angle in degrees of the Hood when the servo is set to 1.0
        public static double hoodServoAngleMax = 360.0;
    }

    @Config
    @Configurable
    public static class Limits {
        // The angle in degrees where the Hood reaches its minimum hard stop
        public static double hoodHardStopMin = 10.0;
        // The angle in degrees where the Hood reaches its maximum hard stop
        public static double hoodHardStopMax = 275.0;
    }

    @Config
    @Configurable
    public static class Control {
        public static double incrementSize = 10.0;
    }
}
