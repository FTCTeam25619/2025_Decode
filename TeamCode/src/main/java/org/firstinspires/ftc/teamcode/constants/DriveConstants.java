package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

public class DriveConstants {
    @Config
    @Configurable
    public static class Joysticks {
        // TODO: Tune these values
        public static double driveSmoothFactor = 1.0;
        public static double driveDeadband = 0.1;
        public static double driveInverseDeadband = 0.05;
        public static double rotateSmoothFactor = 1.0;
        public static double rotateDeadband = 0.1;
        public static double rotateInverseDeadband = 0.05;
        public static double rotateScalingFactor = 1.6;
    }

    @Config
    @Configurable
    public static class HeadingControl {
        public static double kPHeadingCorrection = 0.035;
        public static double correctionOutputClamp = 0.6;
    }
}
