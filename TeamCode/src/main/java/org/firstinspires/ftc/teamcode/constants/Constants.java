package org.firstinspires.ftc.teamcode.constants;

public final class Constants {
    public static final class DebugModes {
        public static final boolean ENABLE_PANELS_DASHBOARD = true;
        public static final class ControlInterface {
            public static final boolean TUNING_CONTROLS = false;
        }
        public static final class Telemetry {
            public static final boolean DEBUG_ODOMETRY = false;
            public static final boolean DEBUG_SHOOTER = false;
            public static final boolean DEBUG_SENSORS = false;
            // Editor complains about simplifying this, but we intentionally want the
            // value to be dependent on whether ENABLE_PANELS_DASHBOARD is true.
            public static final boolean DEBUG_LIGHTS = ENABLE_PANELS_DASHBOARD && false;
            public static final boolean DEBUG_VISION = true;
            public static final boolean DEBUG_PATH_FOLLOWING = false;
        }
    }

    public static final class ConversionFactors {
        public static final double SECONDS_PER_MINUTE = 60.0;
        public static final double FEET_TO_METERS = 0.3048;
        public static final double METERS_TO_FEET = 1.0 / FEET_TO_METERS;
        public static final double MM_TO_INCHES = METERS_TO_FEET * 12.0 / 1000.0;
        public static final double DEGREES_TO_RADIANS = Math.PI / 180.0;
        public static final double RADIANS_TO_DEGREES = 180.0 / Math.PI;
    }
}