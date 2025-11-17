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
        return deadband(smooth(value, smoothFactor), deadband, inverseDeadband);
    }

    public static double deadband(double value, double deadband, double inverseDeadband) {
        // If within deadband, return zero
        if (Math.abs(value) <= deadband) { return 0.0; }
        // Otherwise, return the inverse-deadbanded value
        return (value > 0.0 ? 1.0 : -1.0) * inverseDeadband +
                (1.0 - inverseDeadband) * value;
    }

    public static double smooth(double value, double smoothingFactor) {
        return (smoothingFactor * Math.pow(value, 3.0)) + ((1.0 - smoothingFactor) * value);
    }
}