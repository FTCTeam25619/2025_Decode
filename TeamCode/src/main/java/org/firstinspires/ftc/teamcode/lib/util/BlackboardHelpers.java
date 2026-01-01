package org.firstinspires.ftc.teamcode.lib.util;

import static org.firstinspires.ftc.teamcode.constants.Constants.ConversionFactors.MM_TO_INCHES;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.constants.Alliance;
import org.firstinspires.ftc.teamcode.constants.OdometryConstants;

import java.util.HashMap;

public class BlackboardHelpers {
    // 220.7625mm side to center of rotation
    public static double wallOffsetSideToCenterOfRotationMM = 220.7625;
    // 162.815mm outermost point of intake wheels to center of rotation
    public static double wallOffsetIntakeToCenterOfRotationMM = 162.815;

    public static final class Keys {
        public static final String AUTO_END_POSE = "AUTO_END_POSE";
    }

    public static void setAutoEndPose(HashMap<String, Object> blackboard, Pose pose) {
        blackboard.put(Keys.AUTO_END_POSE, pose);
    }

    public static Pose getAutoEndPoseOrDefault(HashMap<String, Object> blackboard, Alliance alliance) {
        return (Pose) blackboard.getOrDefault(Keys.AUTO_END_POSE, defaultStartingPose(alliance));
    }

    public static void clearAutoEndPose(HashMap<String, Object> blackboard) {
        blackboard.remove(Keys.AUTO_END_POSE);
    }

    private static Pose defaultStartingPose(Alliance alliance) {
        double xIn, yIn;
        double headingRad = Math.toRadians(OdometryConstants.DefaultStartPose.allianceDriveHeadingOffset(alliance));
        if (alliance == Alliance.RED) {
            xIn = wallOffsetIntakeToCenterOfRotationMM * MM_TO_INCHES;
            yIn = wallOffsetSideToCenterOfRotationMM * MM_TO_INCHES;
        } else {
            xIn = 144.0 - wallOffsetIntakeToCenterOfRotationMM * MM_TO_INCHES;
            yIn = wallOffsetSideToCenterOfRotationMM * MM_TO_INCHES;
        }
        return new Pose(xIn, yIn, headingRad);
    }
}
