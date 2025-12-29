package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

public class LiftConstants {
    // FIXME: Tune all of this!
    @Configurable
    public static class DeviceConfig {
        // The angle in degrees of the Lift when the servo is set to 0.0
        public static double liftServoAngleMin = 0.0;
        // The angle in degrees of the Lift when the servo is set to 1.0
        public static double liftServoAngleMax = 300.0;
    }

    @Configurable
    public static class Limits {
        // The angle in degrees where the Lift reaches its minimum hard stop
        public static double liftHardStopMin = 25.0;
        // The angle in degrees where the Lift reaches its maximum hard stop
        public static double liftHardStopMax = 295.0;
    }

    @Configurable
    public static class Control {
        public static double incrementSize = 5.0;
    }

    @Configurable
    public static class Positions {
        public static double engagedDegrees = 30.0;
        public static double disengagedDegrees = 290.0;
        public static double middleOfRange = (Limits.liftHardStopMin + Limits.liftHardStopMax) / 2.0;
        public static double testPositionDegrees =
                (Limits.liftHardStopMin + Limits.liftHardStopMax) / 2.0;
    }
}
