package org.firstinspires.ftc.teamcode.constants;

public class VisionConstants {
    public static class LimelightConfig {
        // These values are not used in code, but are set via the LimeLight configuration
        // tool. They are recorded here for ease of reconfiguration should something go wrong.
        public static double locationForwardMM = 0.0;
        public static double locationRightMM = 0.0;
        public static double locationUpMM = 0.0;
        public static double locationPitchDeg = 0.0;
        public static double locationRollDeg = 0.0;
        public static double locationYawDeg = 0.0;
    }

    public enum AprilTags {
        BlueGoal(20),
        RedGoal(24),
        PatternGPP(21),
        PatternPGP(22),
        PatternPPG(23);

        public final int id;

        private AprilTags(int tagID) {
            id = tagID;
        }
    }
}
