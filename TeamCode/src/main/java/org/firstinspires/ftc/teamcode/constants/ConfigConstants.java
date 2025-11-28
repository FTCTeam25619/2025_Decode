package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

@Config
@Configurable
public class ConfigConstants {
    @Config
    @Configurable
    public static class ManualMovement {
        public static volatile double explorationMotorForward = 0.5;
        public static volatile double explorationMotorBackward = -0.5;
    }

    @Config
    @Configurable
    public static class TestPositions {
    }

    @Config
    @Configurable
    public static class Colors {
        public static volatile float PURPLE_HUE_MIN = 225.0f;
        public static volatile float PURPLE_HUE_MAX = 240.0f;
        public static volatile float GREEN_HUE_MIN = 150.0f;
        public static volatile float GREEN_HUE_MAX = 165.0f;
        public static volatile double MATCHING_DISTANCE_CM = 2.0;
    }
}

