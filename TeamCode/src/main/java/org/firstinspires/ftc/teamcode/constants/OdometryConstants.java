package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

public class OdometryConstants {
    @Configurable
    public static class OdometryPods {
        /*
         * The type of GoBilda Odometry pods used with the PinPoint odometry computer.
         *
         * This is used to set the encoder resolution.
         */
        public static final GoBildaPinpointDriver.GoBildaOdometryPods podType =
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

        /*
         * Directions of each Odometry Pod (assuming GoBilda 4-bar pods)
         */
        public static final GoBildaPinpointDriver.EncoderDirection xDirection =
                GoBildaPinpointDriver.EncoderDirection.FORWARD;
        public static final GoBildaPinpointDriver.EncoderDirection yDirection =
                GoBildaPinpointDriver.EncoderDirection.REVERSED;

        /*
         * Offsets of the Odometry Pods (assuming GoBilda 4-bar pods) relative to the center of the robot.
         *
         * Note: Positive direction of axes depends on the mounting position of the GoBilda PinPoint
         * odometry computer on the robot.
         */
        public static double forwardPodYMM = 79.91;
        public static double strafePodXMM = 140.41;
    }

    @Configurable
    public static class TargetDistances {
        // TODO: Tune real values for these
        public static double atGoalShot = 51;
        public static double nearZoneMidShot = 150;
        public static double nearZoneEndShot = 212;
        public static double farWallShot = 360;
    }
}

