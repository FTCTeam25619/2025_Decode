package org.firstinspires.ftc.teamcode.lib.util;

import com.seattlesolvers.solverslib.util.MathUtils;

import java.lang.Math;

public class NovaPyraUtils {
    public static double[] deadbandAndSmooth(double forward, double strafe, double smoothFactor,
                                      double deadband, double inverseDeadband) {
        // Start with deadbanded (zero) values
        double f = 0.0;
        double s = 0.0;

        // Smooth and deadband the axes via magnitude in polar coordinates
        double magnitude = Math.sqrt((forward * forward) + (strafe * strafe));
        double smoothed = deadbandAndSmoothValue(magnitude, smoothFactor, deadband, inverseDeadband);
        double angle = Math.atan2(strafe, forward);
        double clamped = MathUtils.clamp(smoothed, -1.0, 1.0);
        f = clamped * Math.cos(angle);
        s = clamped * Math.sin(angle);

        // Return an array of smoothed deadbanded values
        return new double[]{f, s};
    }

    public static double deadbandAndSmoothValue(double value, double smoothFactor,
                                                double deadband, double inverseDeadband) {
        return (smooth(deadband(value, deadband), smoothFactor, inverseDeadband));
    }

    public static double deadband(double value, double deadband) {
        // If within deadband, return zero
        return (Math.abs(value) <= deadband ? 0.0 : value);
    }

    public static double smooth(double value, double smoothingFactor, double inverseDeadband) {
        // Shortcut out if zero input
        if (Math.abs(value) <= 0.001) {return 0.0; }

        double smoothed = smoothingFactor * Math.pow(value, 3.0) + ((1.0 - smoothingFactor) * value);
        return (smoothed > 0.0 ? 1.0 : -1.0 ) * inverseDeadband + (1.0 - inverseDeadband) * smoothed;
    }
}