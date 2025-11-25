package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

public class HoodConstants {
    @Configurable
    public static class DeviceConfig {
        // The angle in degrees of the Hood when the servo is set to 0.0
        public static double hoodServoAngleMin = 0.0;
        // The angle in degrees of the Hood when the servo is set to 1.0
        public static double hoodServoAngleMax = 360.0;
    }

    @Configurable
    public static class Limits {
        // The angle in degrees where the Hood reaches its minimum hard stop
        public static double hoodHardStopMin = 45.0;
        // The angle in degrees where the Hood reaches its maximum hard stop
        public static double hoodHardStopMax = 90.0;
    }
}
