package org.firstinspires.ftc.teamcode.lib.util;

import com.seattlesolvers.solverslib.util.MathUtils;

import java.lang.Math;

public class NovaPyraUtils {
    public static double[] deadbandAndSmooth(double forward, double strafe, double rotate,
                                      double deadband, double inverseDeadband,
                                      double moveSmoothFactor, double rotateSmoothFactor) {
        // Start with deadbanded (zero) values
        double f = 0.0;
        double s = 0.0;
        double r = 0.0;

        // If the magnitude is outside the deadband, smooth the axes via polar coordinates
        double magnitude = Math.sqrt((forward * forward) + (strafe * strafe));
        if (magnitude > deadband) {
            double angle = Math.atan2(strafe, forward);
            double smoothed = MathUtils.clamp(smooth(magnitude, moveSmoothFactor), -1.0, 1.0);
            f = smoothed * Math.cos(angle);
            s = smoothed * Math.sin(angle);
        }
        // If the rotation axis is outside the deadband, apply inverse deadband and smoothing
        if (Math.abs(rotate) > deadband) {
            r = (rotate > 0.0 ? 1.0 : -1.0) * inverseDeadband +
                    (1.0 - inverseDeadband) * smooth(rotate, rotateSmoothFactor);
        }

        // Return an array of smoothed deadbanded values
        return new double[]{f, s, r};
    }

    public static double smooth(double value, double smoothingFactor) {
        return (smoothingFactor * Math.pow(value, 3.0)) + ((1.0 - smoothingFactor) * value);
    }
}