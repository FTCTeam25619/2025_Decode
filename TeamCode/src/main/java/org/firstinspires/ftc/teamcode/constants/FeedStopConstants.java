package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

@Config
@Configurable
public class FeedStopConstants {
    public static double engagedDegrees = 250.0;
    public static double disengagedDegrees = 5.0;
    public static volatile double testPositionDegrees = 100.0;
    public static double minLimitDegrees = 0.0;
    public static double maxLimitDegrees = 270.0;
}
