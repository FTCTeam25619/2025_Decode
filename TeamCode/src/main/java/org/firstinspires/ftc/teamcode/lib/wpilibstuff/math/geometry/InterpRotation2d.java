package org.firstinspires.ftc.teamcode.lib.wpilibstuff.math.geometry;

import com.seattlesolvers.solverslib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.lib.wpilibstuff.math.MathUtil;
import org.firstinspires.ftc.teamcode.lib.wpilibstuff.math.interpolation.Interpolatable;

import java.util.Objects;

public class InterpRotation2d extends Rotation2d implements Interpolatable<InterpRotation2d> {

    public static final InterpRotation2d kZero = new InterpRotation2d();

    public InterpRotation2d roteteBy(Rotation2d other) {
        return (InterpRotation2d) super.rotateBy(other);
    }

    public InterpRotation2d plus(Rotation2d other) {
        return (InterpRotation2d) super.plus(other);
    }

    public InterpRotation2d minus(Rotation2d other) {
        return (InterpRotation2d) super.minus(other);
    }

    public InterpRotation2d times(double scalar) {
        return (InterpRotation2d) super.times(scalar);
    }
    public InterpRotation2d interpolate(InterpRotation2d endValue, double t) {
        return plus(endValue.minus(this).times(MathUtil.clamp(t, 0, 1)));
    }
}
