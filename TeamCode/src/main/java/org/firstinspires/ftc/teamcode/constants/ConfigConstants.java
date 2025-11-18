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
}

