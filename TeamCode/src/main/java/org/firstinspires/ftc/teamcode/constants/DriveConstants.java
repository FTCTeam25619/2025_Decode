package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

public class DriveConstants {
    @Config
    @Configurable
    public static class Joysticks {
        // TODO: Tune these values
        public static double driveSmoothFactor = 0.9;
        public static double driveDeadband = 0.1;
        public static double driveInverseDeadband = 0.1;
        public static double rotateSmoothFactor = 0.95;
        public static double rotateDeadband = 0.15;
        public static double rotateInverseDeadband = 0.15;
    }
}
