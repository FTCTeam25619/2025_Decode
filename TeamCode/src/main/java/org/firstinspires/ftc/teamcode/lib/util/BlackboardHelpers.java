package org.firstinspires.ftc.teamcode.lib.util;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.constants.Alliance;
import org.firstinspires.ftc.teamcode.constants.OdometryConstants;

import java.util.HashMap;

public class BlackboardHelpers {
    public static final class Keys {
        public static final String AUTO_END_POSE = "AUTO_END_POSE";
    }

    public static void setAutoEndPose(HashMap<String, Object> blackboard, Pose pose) {
        blackboard.put(Keys.AUTO_END_POSE, pose);
    }

    public static Pose getAutoEndPoseOrDefault(HashMap<String, Object> blackboard, Alliance alliance) {
        Pose defaultStartingPose = new Pose(0.0, 0.0,
                Math.toRadians(OdometryConstants.DefaultStartPose.allianceDriveHeadingOffset(alliance)));
        return (Pose) blackboard.getOrDefault(Keys.AUTO_END_POSE, defaultStartingPose);
    }

    public static void clearAutoEndPose(HashMap<String, Object> blackboard) {
        blackboard.remove(Keys.AUTO_END_POSE);
    }
}
