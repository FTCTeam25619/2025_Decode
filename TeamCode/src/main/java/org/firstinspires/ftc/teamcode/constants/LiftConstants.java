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

    public static class Limits {
        @Configurable
        public static class LeftServo {
            // The angle in degrees where the Lift left servo reaches its minimum hard stop
            public static double hardStopMin = 0.0;
            // The angle in degrees where the Lift right servo reaches its maximum hard stop
            public static double hardStopMax = 180.0;
        }
        @Configurable
        public static class RightServo {
            // The angle in degrees where the Lift left servo reaches its minimum hard stop
            public static double hardStopMin = 0.0;
            // The angle in degrees where the Lift right servo reaches its maximum hard stop
            public static double hardStopMax = 180.0;
        }
    }

    @Configurable
    public static class Control {
        public static double incrementSize = 5.0;
    }

    public static class Positions {
        @Configurable
        public static class LeftServo {
            public static double engagedDegrees = 60.0;
            public static double disengagedDegrees = 160.0;
            public static double middleOfRange =
                    (Limits.LeftServo.hardStopMin + Limits.LeftServo.hardStopMax) / 2.0;
            public static double testPositionDegrees =
                    (Limits.LeftServo.hardStopMin + Limits.LeftServo.hardStopMax) / 2.0;

        }
        @Configurable
        public static class RightServo {
            public static double engagedDegrees = 60.0;
            public static double disengagedDegrees = 160.0;
            public static double middleOfRange =
                    (Limits.RightServo.hardStopMin + Limits.RightServo.hardStopMax) / 2.0;
            public static double testPositionDegrees =
                    (Limits.RightServo.hardStopMin + Limits.RightServo.hardStopMax) / 2.0;

        }
    }
}
