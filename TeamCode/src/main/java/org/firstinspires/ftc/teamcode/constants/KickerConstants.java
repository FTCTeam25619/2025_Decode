package org.firstinspires.ftc.teamcode.constants;


import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class KickerConstants {

    public static double rpmTolerance = 160.0;
    public static double minRPM = 0.0;
    public static double maxRPM = 6000.0;
    public static final double gearRatio = 1.0;
    public static boolean motorInverted = false;
    public static double flywheelRPMMultiplier = 1.5;

    @Configurable
    public static class VelocityPID {
        public static double kP = 30.0;
        public static double kI = 0.0;
        public static double kD = 5.0;
        public static double kF = 14.0;
    }
    @Configurable
    public static class Targets {
        public static double idleRPM = 2800.0;

    }


}
