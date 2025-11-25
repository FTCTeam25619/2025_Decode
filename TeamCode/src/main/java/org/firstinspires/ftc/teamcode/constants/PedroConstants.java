package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PedroConstants {
    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .leftFrontMotorName(Devices.Drivetrain.driveMotorFrontLeft)
            .leftRearMotorName(Devices.Drivetrain.driveMotorBackLeft)
            .rightFrontMotorName(Devices.Drivetrain.driveMotorFrontRight)
            .rightRearMotorName(Devices.Drivetrain.driveMotorBackRight)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(OdometryConstants.forwardPodYMM)
            .strafePodX(OdometryConstants.strafePodXMM)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName(Devices.Odometry.pinpointOdometry)
            .encoderResolution(OdometryConstants.podType)
            .forwardEncoderDirection(OdometryConstants.yDirection)
            .strafeEncoderDirection(OdometryConstants.xDirection);

    public static Drivetrain drivetrain;
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(4.92);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99, 100, 1, 1);

    public static Drivetrain createDrivetrain(HardwareMap hardwareMap) {
        if (drivetrain == null) {
            drivetrain = new Mecanum(hardwareMap, mecanumConstants);
        }
        return drivetrain;
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .setDrivetrain(createDrivetrain(hardwareMap))
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
