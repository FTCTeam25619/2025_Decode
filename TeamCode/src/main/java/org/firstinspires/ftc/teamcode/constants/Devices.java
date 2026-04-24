package org.firstinspires.ftc.teamcode.constants;

public class Devices {
    public static final class Power {
        public static final String currentSensor = "current_sensor";
    }

    public static final class Drivetrain {
        public static final String driveMotorFrontLeft = "drive_motor_front_left";
        public static final String driveMotorFrontRight = "drive_motor_front_right";
        public static final String driveMotorBackLeft = "drive_motor_back_left";
        public static final String driveMotorBackRight = "drive_motor_back_right";
    }

    public static final class Lift {
        public static final String liftServoLeft = "servo_lift_left";
        public static final String liftServoRight = "servo_lift_right";
    }

    public static final class Indicators {
        public static final String indicatorLight0 = "indicator_light_0";
        public static final String indicatorLight1 = "indicator_light_1";
        public static final String indicatorLight2 = "indicator_light_2";
        public static final String indicatorLight3 = "indicator_light_3";
    }

    public static final class Sensors {
        //public static final String intakeColorSensor = "intake_color_sensor";
        //public static final String feedColorSensorLow = "feed_color_sensor_low";
        public static final String feedColorSensorHigh = "feed_color_sensor_high";
    }

    public static final class Vision {
        public static final String limelightCamera = "limelight_camera";
    }

    public static final class Intake {
        public static final String intakeMotor = "intake_motor";
    }

    public static final class Hood {
        public static final String hoodServo = "servo_hood";
        public static final String hoodEncoder = "hood_encoder";
    }

    public static final class ShooterFeed {
        public static final String feedCRServoLeft = "crservo_feed_left";
        public static final String feedCRServoRight = "crservo_feed_right";
        public static final String feedEncoderLeft = "feed_left_encoder";
        public static final String feedEncoderRight = "feed_right_encoder";
    }

    public static final class FeedStop {
        public static final String feedStopServo = "servo_feed_stop";
    }

    public static final class Shooter {
        public static final String leftMotor = "shooter_motor_left";
        public static final String rightMotor = "shooter_motor_right";
    }

    public static final class Odometry {
        public static final String pinpointOdometry = "pinpoint_odometry_cpu";
    }
}
