package org.firstinspires.ftc.teamcode.constants;

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
    public static double forwardPodYMM = 79.91;
    public static double strafePodXMM = 140.41;

    /*
     * Directions of each Odometry Pod (assuming GoBilda 4-bar pods)
     */
    public static final GoBildaPinpointDriver.EncoderDirection xDirection =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final GoBildaPinpointDriver.EncoderDirection yDirection =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;
}
