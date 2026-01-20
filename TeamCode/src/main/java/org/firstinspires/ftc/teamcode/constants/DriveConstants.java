package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

public class DriveConstants {
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

    @Configurable
    public static class HeadingControl {
        @Configurable
        public static class PrimaryPID {

            public static double kPHeadingCorrection = 0.015;
            public static double kIHeadingCorrection = 0.0;
            public static double kDHeadingCorrection = 0.0025;
        }
        @Configurable
        public static class SecondaryPID {

            public static double kPSecondaryHeadingCorrection = 0.01;
            public static double kISecondaryHeadingCorrection = 0.1;
            public static double kDSecondaryHeadingCorrection = 0.001;
        }
        public static double correctionOutputClamp = 0.4;
        public static double kFHeadingCorrection = 0.025;
        public static double SecondaryHeadingThreshold = 3.5;
    }

    @Configurable
    public static class PoseEstimator {
        public static boolean useVisionPoseEstimate = true;  // If enabled apply vision updates to pose
        public static double[] stateStdDevs = {0.05, 0.05, 0.0};
        public static double[] visionStdDevs = {0.1, 0.1, 0.5};
        public static double maxSlewRateDPS = 360.0;
    }
}
