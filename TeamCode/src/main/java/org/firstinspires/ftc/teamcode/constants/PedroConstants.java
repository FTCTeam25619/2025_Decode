package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.Drivetrain;
import com.pedropathing.control.PIDFCoefficients;
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
import org.firstinspires.ftc.teamcode.lib.pedroExtensions.PoseEstFollower;
import org.firstinspires.ftc.teamcode.lib.pedroExtensions.PoseEstFollowerBuilder;

public class PedroConstants {
    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .leftFrontMotorName(Devices.Drivetrain.driveMotorFrontLeft)
            .leftRearMotorName(Devices.Drivetrain.driveMotorBackLeft)
            .rightFrontMotorName(Devices.Drivetrain.driveMotorFrontRight)
            .rightRearMotorName(Devices.Drivetrain.driveMotorBackRight)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true)
            .xVelocity(71.891427)
            .yVelocity(50.062855);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(OdometryConstants.OdometryPods.forwardPodYMM)
            .strafePodX(OdometryConstants.OdometryPods.strafePodXMM)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName(Devices.Odometry.pinpointOdometry)
            .encoderResolution(OdometryConstants.OdometryPods.podType)
            .forwardEncoderDirection(OdometryConstants.OdometryPods.yDirection)
            .strafeEncoderDirection(OdometryConstants.OdometryPods.xDirection);

    public static Drivetrain drivetrain;
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.245)
            .forwardZeroPowerAcceleration(-31.295587)
            .lateralZeroPowerAcceleration(-70.547653)
            .useSecondaryTranslationalPIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.12, 0.0, 0.01, 0.028))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.18, 0.0, 0.01, 0.035))
            .useSecondaryHeadingPIDF(true)
            .headingPIDFCoefficients(new PIDFCoefficients(1.0, 0.0, 0.025, 0.028))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2.2, 0.0, 0.025, 0.08))
            // TODO: Centripetal scaling not yet retuned. Stopped before Drive Tuner.
            .centripetalScaling(0.0012);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99, 100, 0.3, 2.5);

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

    public static PoseEstFollower createPoseEstFollower(HardwareMap hardwareMap) {
        return new PoseEstFollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .setDrivetrain(createDrivetrain(hardwareMap))
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
