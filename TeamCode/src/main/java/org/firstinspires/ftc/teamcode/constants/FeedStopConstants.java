package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

public class FeedStopConstants {
    @Configurable
    public static class DeviceConfig {
        // The angle in degrees of the Hood when the servo is set to 0.0
        public static double feedStopServoAngleMin = 0.0;
        // The angle in degrees of the Hood when the servo is set to 1.0
        public static double feedStopServoAngleMax = 300.0;
    }

    @Configurable
    public static class Limits {
        // The angle in degrees where the Hood reaches its minimum hard stop
        public static double feedStopHardStopMin = 0.0;
        // The angle in degrees where the Hood reaches its maximum hard stop
        public static double feedStopHardStopMax = 300.0;
    }

    @Configurable
    public static class Control {
        public static double incrementSize = 10.0;
    }

    @Configurable
    public static class Positions {
        public static double engagedDegrees = Limits.feedStopHardStopMax;
        public static double disengagedDegrees = Limits.feedStopHardStopMin;
        public static double testPositionDegrees =
                (Limits.feedStopHardStopMin + Limits.feedStopHardStopMax) / 2.0;
    }
}
