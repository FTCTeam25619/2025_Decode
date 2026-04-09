package org.firstinspires.ftc.teamcode.constants;



import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class VisionConstants {
    @Configurable
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



    public static double headingAngleThresholdDeg = 1.5;


    @Configurable
    public static class targetOffsets{
        // Tunable offsets for aiming adjustments
        public static double blueGoalXOffset = 7.0;      // inches
        public static double blueGoalYOffset = 143.0;      // inches
        public static double redGoalXOffset = 137.0;       // inches
        public static double redGoalYOffset = 143.0;       // inches
    }

    public enum AprilTags {
        BlueGoal(20, new Pose(0, 0, 0.0)),
        RedGoal(24, new Pose(0, 0, 0.0)),
        PatternGPP(21, new Pose(0.0, 0.0, 0.0)),
        PatternPGP(22, new Pose(0.0, 0.0, 0.0)),
        PatternPPG(23, new Pose(0.0, 0.0, 0.0));

        public final int id;
        public final Pose pose;

        private AprilTags(int tagID, Pose tagCoords) {
            id = tagID;
            pose = tagCoords;
        }
    }
}
