// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.lib.wpilibstuff.math;

/** Math utility functions. */
public final class MathUtil {
    private MathUtil() {
        throw new AssertionError("utility class");
    }

    /**
     * Returns value clamped between low and high boundaries.
     *
     * @param value Value to clamp.
     * @param low The lower boundary to which to clamp value.
     * @param high The higher boundary to which to clamp value.
     * @return The clamped value.
     */
    public static int clamp(int value, int low, int high) {
        return Math.max(low, Math.min(value, high));
    }
    /**
     * Returns value clamped between low and high boundaries.
     *
     * @param value Value to clamp.
     * @param low The lower boundary to which to clamp value.
     * @param high The higher boundary to which to clamp value.
     * @return The clamped value.
     */
    public static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and the maximum magnitude is scaled from 0.0 to the maximum magnitude.
     *
     * @param value Value to clip.
     * @param deadband Range around zero.
     * @param maxMagnitude The maximum magnitude of the input. Can be infinite.
     * @return The value after the deadband is applied.
     */


    /**
     * Returns modulus of input.
     *
     * @param input        Input value to wrap.
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     * @return The wrapped value.
     */
    public static double inputModulus(double input, double minimumInput, double maximumInput) {
        double modulus = maximumInput - minimumInput;

        // Wrap input if it's above the maximum input
        int numMax = (int) ((input - minimumInput) / modulus);
        input -= numMax * modulus;

        // Wrap input if it's below the minimum input
        int numMin = (int) ((input - maximumInput) / modulus);
        input -= numMin * modulus;

        return input;
    }

    /**
     * Wraps an angle to the range -pi to pi radians.
     *
     * @param angleRadians Angle to wrap in radians.
     * @return The wrapped angle.
     */
    public static double angleModulus(double angleRadians) {
        return inputModulus(angleRadians, -Math.PI, Math.PI);
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
        return startValue + (endValue - startValue) * MathUtil.clamp(t, 0, 1);
    }

    /**
     * Return where within interpolation range [0, 1] q is between startValue and endValue.
     *
     * @param startValue Lower part of interpolation range.
     * @param endValue   Upper part of interpolation range.
     * @param q          Query.
     * @return Interpolant in range [0, 1].
     */
    public static double inverseInterpolate(double startValue, double endValue, double q) {
        double totalRange = endValue - startValue;
        if (totalRange <= 0) {
            return 0.0;
        }
        double queryToStart = q - startValue;
        if (queryToStart <= 0) {
            return 0.0;
        }
        return queryToStart / totalRange;
    }
}
