package org.firstinspires.ftc.teamcode.lib.wpilibstuff.math.geometry;

import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Transform2d;
import com.seattlesolvers.solverslib.geometry.Twist2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.wpilibstuff.math.interpolation.Interpolatable;

public class InterpPose2d extends Pose2d implements Interpolatable<InterpPose2d> {
    /**
     * A preallocated Pose2d representing the origin.
     *
     * <p>This exists to avoid allocations for common poses.
     */
    public static final InterpPose2d kZero = new InterpPose2d();

    public InterpPose2d() {
        super();
    }

    @Override
    public InterpPose2d plus(Transform2d other) {
        return (InterpPose2d) super.plus(other);
    }

    public InterpPose2d tansformBy(Transform2d other) {
        return (InterpPose2d) super.transformBy(other);
    }

    public InterpPose2d relativeTo(Pose2d other) {
        return (InterpPose2d) super.relativeTo(other);
    }

    public InterpPose2d exp(Twist2d twist) {
        return (InterpPose2d) super.exp(twist);
    }

    public InterpPose2d rotate(double deltaTheta) {
        return (InterpPose2d) super.rotate(deltaTheta);
    }

    public InterpPose2d interpolate(InterpPose2d endValue, double t) {
        if (t < 0) {
            return this;
        } else if (t >= 1) {
            return endValue;
        } else {
            Twist2d twist = this.log(endValue);
            Twist2d scaledTwist = new Twist2d(twist.dx * t, twist.dy * t, twist.dtheta * t);
            return this.exp(scaledTwist);
        }
    }

}
