package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.Devices;
import org.firstinspires.ftc.teamcode.constants.OdometryConstants;

import java.util.Locale;

public class Odometry extends SubsystemBase {
    private final GoBildaPinpointDriver odometry;
    private final Telemetry telemetry;

    private boolean trackingCycleTime = false;
    private double priorRevHubClockReading;
    private double cyclePeriod;
    private double cycleFrequency;

    public Odometry(HardwareMap hardwareMap, Telemetry telemetry) {
        this.odometry = hardwareMap.get(GoBildaPinpointDriver.class, Devices.Odometry.pinpointOdometry);
        this.telemetry = telemetry;

        setupOdometry();
    }

    private void setupOdometry() {
        this.odometry.setOffsets(OdometryConstants.xOffsetMM, OdometryConstants.yOffsetMM, DistanceUnit.MM);
        this.odometry.setEncoderResolution(OdometryConstants.podType);
        this.odometry.setEncoderDirections(OdometryConstants.xDirection, OdometryConstants.yDirection);
        this.odometry.resetPosAndIMU();

        if (Constants.DebugModes.Telemetry.DEBUG_ODOMETRY) {
            telemetry.addData("Status", "Initialized");
            telemetry.addData("X offset (mm)", this.odometry.getXOffset(DistanceUnit.MM));
            telemetry.addData("Y offset (mm)", this.odometry.getYOffset(DistanceUnit.MM));
            telemetry.addData("Device Version Number:", this.odometry.getDeviceVersion());
            telemetry.addData("Heading Scalar", this.odometry.getYawScalar());
            telemetry.update();
        }
    }

    @Override
    public void periodic() {
        this.odometry.update();
        addOdometryTelemetry();
    }

    public void setOdometryHeading(double heading, AngleUnit angleUnit) {
        this.odometry.setHeading(heading, angleUnit);
    }

    public void setRobotPose(Pose2D pose) {
        this.odometry.setPosition(pose);
    }

    public void resetOdometry() {
        this.odometry.resetPosAndIMU();
    }

    public void recalibrateOdometryIMU() {
        this.odometry.recalibrateIMU();
    }

    public void updateRevHubCycleFrequency(double newClockReading) {
        trackingCycleTime = true;
        cyclePeriod = newClockReading - priorRevHubClockReading;
        cycleFrequency = 1/cyclePeriod;
        priorRevHubClockReading = newClockReading;
    }

    private void addOdometryTelemetry() {
        if (Constants.DebugModes.Telemetry.DEBUG_ODOMETRY) {
            /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
            Pose2D pos = this.odometry.getPosition();
            String data = String.format(Locale.US,
                    "{X (mm): %.3f, Y (mm): %.3f, H (deg): %.3f}",
                    pos.getX(DistanceUnit.MM),
                    pos.getY(DistanceUnit.MM),
                    pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
            String velocity = String.format(Locale.US,
                    "{XVel (mm/s): %.3f, YVel (mm/s): %.3f, HVel (deg/s): %.3f}",
                    this.odometry.getVelX(DistanceUnit.MM),
                    this.odometry.getVelY(DistanceUnit.MM),
                    this.odometry.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);

            /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            FAULT_BAD_READ - The firmware detected a bad I²C read, if a bad read is detected, the device status is updated and the previous position is reported
            */
            telemetry.addData("Status", this.odometry.getDeviceStatus().toString());

            // prints/gets the current refresh rate of the Pinpoint
            double pinpointFrequency = this.odometry.getFrequency();
            telemetry.addData("Pinpoint Refresh Period (ms)", 1000.0/pinpointFrequency);
            telemetry.addData("Pinpoint Frequency", pinpointFrequency);

            if (trackingCycleTime) {
                // prints the control system refresh rate
                telemetry.addData("REV Hub Frequency: ", cycleFrequency);
                telemetry.addData("REV Hub Cycle Time (ms): ", cyclePeriod * 1000.0);
            }

            telemetry.update();
        }
    }
}
