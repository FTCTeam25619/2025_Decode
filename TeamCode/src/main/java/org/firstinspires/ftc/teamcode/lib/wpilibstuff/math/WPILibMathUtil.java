// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.lib.wpilibstuff.math;

import com.seattlesolvers.solverslib.util.MathUtils;

/** Math utility functions. */
public final class WPILibMathUtil {
    private WPILibMathUtil() {
        throw new AssertionError("utility class");
    }

    /**
     * Perform linear interpolation between two values.
     *
     * @param startValue The value to start at.
     * @param endValue   The value to end at.
     * @param t          How far between the two values to interpolate. This is clamped to [0, 1].
     * @return The interpolated value.
     */
    public static double interpolate(double startValue, double endValue, double t) {
        return startValue + (endValue - startValue) * MathUtils.clamp(t, 0, 1);
    }

}
