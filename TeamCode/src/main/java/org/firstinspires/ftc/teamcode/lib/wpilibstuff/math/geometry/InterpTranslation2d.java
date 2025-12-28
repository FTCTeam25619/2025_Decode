package org.firstinspires.ftc.teamcode.lib.wpilibstuff.math.geometry;

import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.lib.wpilibstuff.math.MathUtil;
import org.firstinspires.ftc.teamcode.lib.wpilibstuff.math.interpolation.Interpolatable;

public class InterpTranslation2d extends Translation2d implements Interpolatable<InterpTranslation2d> {

    public static final InterpTranslation2d kZero = new InterpTranslation2d(0.0, 0.0);

    public InterpTranslation2d(double x, double y) {
        super(x, y);
    }

    public InterpTranslation2d rotateBy(Rotation2d other) {
        return (InterpTranslation2d) super.rotateBy(other);
    }

    public InterpTranslation2d plus(InterpTranslation2d other) {
        return (InterpTranslation2d) super.plus(other);
    }

    public InterpTranslation2d minus(InterpTranslation2d other) {
        return (InterpTranslation2d) super.minus(other);
    }

    public InterpTranslation2d unaryMinus() {
        return (InterpTranslation2d) super.unaryMinus();
    }

    public InterpTranslation2d times(double scalar) {
        return (InterpTranslation2d) super.times(scalar);
    }

    public InterpTranslation2d div(double scalar) {
        return (InterpTranslation2d) super.div(scalar);
    }

    public InterpTranslation2d interpolate(InterpTranslation2d endValue, double t) {
        double clampedT = MathUtil.clamp(t, 0.0, 1.0);
        return new InterpTranslation2d(
                MathUtil.interpolate(this.getX(), endValue.getX(), clampedT),
                MathUtil.interpolate(this.getY(), endValue.getY(), clampedT)
        );
    }
}
