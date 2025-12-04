package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

public class OdometryConstants {
    /*
     * The type of GoBilda Odometry pods used with the PinPoint odometry computer.
     *
     * This is used to set the encoder resolution.
     */
    public static final GoBildaPinpointDriver.GoBildaOdometryPods podType =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    /*
     * Offsets of the Odometry Pods (assuming GoBilda 4-bar pods) relative to the center of the robot.
     *
     * Note: Positive direction of axes depends on the mounting position of the GoBilda PinPoint
     * odometry computer on the robot.
     */
    public static double xOffsetMM = 72.0;
    public static double yOffsetMM = 136.0;

    /*
     * Directions of each Odometry Pod (assuming GoBilda 4-bar pods)
     */
    public static final GoBildaPinpointDriver.EncoderDirection xDirection =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static final GoBildaPinpointDriver.EncoderDirection yDirection =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    @Config
    @Configurable
    public static class TargetDistances {
        // TODO: Tune real values for these
        public static double atGoalShot = 0.2;
        public static double nearZoneMidShot = 1.0;
        public static double nearZoneEndShot = 2.0;
        public static double farWallShot = 3.5;
    }
}
