/**
 * Enhanced version of Pose that adds interpolation as well as translation, rotation and twist objects
 * and related math from WPILib.  Needed for Pose Estimation.
 *
 * Includes getters and setters for "normal" PedroPathing poses.
 *
 * Ideally this would have been a wrapper for the PedroPathing class Pose, but can't as PedroPathing
 * declared class Pose as final.  Arrrgh!
 */

package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.lib.wpilibstuff.math.interpolation.Interpolatable;

public class InterpPose implements FuturePose, Interpolatable<InterpPose> {

    private final double x;
    private final double y;
    private final double heading;
    private final CoordinateSystem coordinateSystem;

    /**
     * Constructs a pose with the specified position, heading, and coordinate system.
     *
     * @param x the x-coordinate
     * @param y the y-coordinate
     * @param heading the heading in radians
     * @param coordinateSystem the coordinate system
     */
    public InterpPose(double x, double y, double heading, CoordinateSystem coordinateSystem) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.coordinateSystem = coordinateSystem;

    }

    /**
     * Constructs a pose with the specified position and heading, using the default Pedro coordinate system.
     *
     * @param x the x-coordinate
     * @param y the y-coordinate
     * @param heading the heading in radians
     */
    public InterpPose(double x, double y, double heading) {
        this(x, y, heading, PedroCoordinates.INSTANCE);
    }

    /**
     * Constructs a pose with the specified position and a heading of 0, using the default Pedro coordinate system.
     *
     * @param x the x-coordinate
     * @param y the y-coordinate
     */
    public InterpPose(double x, double y) {
        this(x, y, 0);
    }

    /**
     * Constructs a pose at the origin (0, 0) with heading 0, using the default Pedro coordinate system.
     */
    public InterpPose() {
        this(0,0,0);
    }

    public InterpPose(Pose pose) {
        this.x = pose.getX();
        this.y = pose.getY();
        this.heading = pose.getHeading();
        this.coordinateSystem = pose.getCoordinateSystem();
    }

    /**
     * Returns the x-coordinate of the pose.
     *
     * @return the x-coordinate
     */
    public double getX() {
        return x;
    }

    /**
     * Returns the y-coordinate of the pose.
     *
     * @return the y-coordinate
     */
    public double getY() {
        return y;
    }

    /**
     * Returns the heading of the pose in radians.
     *
     * @return the heading in radians
     */
    public double getHeading() {
        return heading;
    }

    /**
     * Returns the coordinate system of the pose.
     *
     * @return the coordinate system
     */
    public CoordinateSystem getCoordinateSystem() {
        return coordinateSystem;
    }

    /**
     * Returns a new pose with the specified x-coordinate, keeping other values the same.
     *
     * @param x the new x-coordinate
     * @return a new pose with updated x
     */
    public InterpPose withX(double x) {
        return new InterpPose(x, y, heading, coordinateSystem);
    }

    /**
     * Returns a new pose with the specified y-coordinate, keeping other values the same.
     *
     * @param y the new y-coordinate
     * @return a new pose with updated y
     */
    public InterpPose withY(double y) {
        return new InterpPose(x, y, heading, coordinateSystem);
    }

    /**
     * Returns a new pose with the specified heading, keeping other values the same.
     *
     * @param heading the new heading in radians
     * @return a new pose with updated heading
     */
    public InterpPose withHeading(double heading) {
        return new InterpPose(x, y, heading, coordinateSystem);
    }

    /**
     * Returns the position of the pose as a \`Vector\`.
     *
     * @return a vector representing the position
     */
    public Vector getAsVector() {
        Vector vector = new Vector();
        vector.setOrthogonalComponents(x, y);
        return vector;
    }

    /**
     * Returns the heading of the pose as a unit vector.
     *
     * @return a unit vector in the direction of the heading
     */
    public Vector getHeadingAsUnitVector() {
        return new Vector(1, heading);
    }

    /**
     * Adds another pose to this pose, converting coordinate systems if necessary.
     *
     * @param other the pose to add
     * @return the resulting pose
     */
    public InterpPose plus(InterpPose other) {
        InterpPose inCurrentCoordinates = coordinateSystem == other.coordinateSystem ? other :
                other.getAsCoordinateSystem(
                        coordinateSystem);
        return new InterpPose(x + inCurrentCoordinates.x,
                y + inCurrentCoordinates.y,
                heading + inCurrentCoordinates.heading,
                coordinateSystem);
    }

    /**
     * Adds another pose to this pose, converting coordinate systems if necessary.
     *
     * @param other the pose to add
     * @return the resulting pose
     */
    public InterpPose plus(Pose other) {
        Pose inCurrentCoordinates = this.coordinateSystem == other.getCoordinateSystem() ?
                other :
                other.getAsCoordinateSystem(
                        coordinateSystem);
        return new InterpPose(x + inCurrentCoordinates.getX(),
                y + inCurrentCoordinates.getY(),
                heading + inCurrentCoordinates.getHeading(),
                coordinateSystem);
    }

    /**
     * Subtracts another pose from this pose, converting coordinate systems if necessary.
     *
     * @param other the pose to subtract
     * @return the resulting pose
     */
    public InterpPose minus(InterpPose other) {
        InterpPose inCurrentCoordinates = coordinateSystem == other.coordinateSystem ? other :
                other.getAsCoordinateSystem(
                        coordinateSystem);
        return new InterpPose(x - inCurrentCoordinates.x,
                y - inCurrentCoordinates.y,
                heading - inCurrentCoordinates.heading,
                coordinateSystem);
    }

    /**
     * Subtracts another pose from this pose, converting coordinate systems if necessary.
     *
     * @param other the pose to subtract
     * @return the resulting pose
     */
    public InterpPose minus(Pose other) {
        Pose inCurrentCoordinates = coordinateSystem == other.getCoordinateSystem() ? other :
                other.getAsCoordinateSystem(
                        coordinateSystem);
        return new InterpPose(x - inCurrentCoordinates.getX(),
                y - inCurrentCoordinates.getY(),
                heading - inCurrentCoordinates.getHeading(),
                coordinateSystem);
    }

    /**
     * Multiplies the pose by a scalar.
     * This scales x, y, and heading by the scalar value, but does not change the coordinate system.
     *
     * @param scalar the scalar value
     * @return the resulting pose
     */
    public InterpPose times(double scalar) {
        return new InterpPose(
                x * scalar,
                y * scalar,
                heading * scalar,
                coordinateSystem
        );
    }

    /**
     * This scales the current Pose by a given scale factor.
     * The heading or coordinate system is not affected.
     *
     * @param scalar the scale factor
     * @return returns the scaled Pose
     */
    public InterpPose scale(double scalar) {
        return new InterpPose(getX() * scalar, getY() * scalar, getHeading(), coordinateSystem);
    }

    /**
     * Divides the pose by a scalar.
     *
     * @param scalar the scalar value
     * @return the resulting pose
     */
    public InterpPose div(double scalar) {
        return new InterpPose(
                x / scalar,
                y / scalar,
                heading / scalar,
                coordinateSystem
        );
    }

    /**
     * Returns the negation of this pose.
     *
     * @return a pose with all components negated
     */
    public InterpPose unaryMinus() {
        return new InterpPose(-x, -y, -heading, coordinateSystem);
    }


    /**
     * This returns a Pose that is the linear combination of the current pose and another pose.
     * @param other the other pose
     * @param scaleThis the coefficient for the current pose
     * @param scaleOther the second coefficient for the other pose
     * @return the resulting pose
     */
    public InterpPose linearCombination(InterpPose other, double scaleThis, double scaleOther) {
        return times(scaleThis).plus(other.times(scaleOther));
    }

    /**
     * This returns a Pose that is the linear combination of the current pose and another pose.
     * @param other the other pose
     * @param scaleThis the coefficient for the current pose
     * @param scaleOther the second coefficient for the other pose
     * @return the resulting pose
     */
    public InterpPose linearCombination(Pose other, double scaleThis, double scaleOther) {
        return times(scaleThis).plus(other.times(scaleOther));
    }


    /**
     * Checks if this pose is approximately equal to another pose within a given accuracy.
     *
     * @param other the other pose
     * @param accuracy the allowed difference
     * @return true if poses are approximately equal, false otherwise
     */
    public boolean roughlyEquals(InterpPose other, double accuracy) {
        return MathFunctions.roughlyEquals(x, other.x, accuracy) &&
                MathFunctions.roughlyEquals(y, other.y, accuracy) &&
                MathFunctions.roughlyEquals(MathFunctions.getSmallestAngleDifference(heading,
                        other.heading), 0, accuracy);
    }

    /**
     * Checks if this pose is approximately equal to another pose within a given accuracy.
     *
     * @param other the other pose
     * @param accuracy the allowed difference
     * @return true if poses are approximately equal, false otherwise
     */
    public boolean roughlyEquals(Pose other, double accuracy) {
        return MathFunctions.roughlyEquals(x, other.getX(), accuracy) &&
                MathFunctions.roughlyEquals(y, other.getY(), accuracy) &&
                MathFunctions.roughlyEquals(MathFunctions.getSmallestAngleDifference(heading,
                        other.getHeading()), 0, accuracy);
    }

    /**
     * Calculates the Euclidean distance from this pose to another pose.
     *
     * @param other the other pose
     * @return the distance between the two poses
     */
    public double distanceFrom(InterpPose other) {
        return Math.sqrt(Math.pow(other.x - x, 2) + Math.pow(other.y - y, 2));
    }

    /**
     * Calculates the Euclidean distance from this pose to another pose.
     *
     * @param other the other pose
     * @return the distance between the two poses
     */
    public double distanceFrom(Pose other) {
        return Math.sqrt(Math.pow(other.getX() - x, 2) + Math.pow(other.getY() - y, 2));
    }

    /**
     * Calculates the squared Euclidean distance from this pose to another pose.
     * @param other the other pose
     * @return the squared distance between the two poses
     */
    public double distSquared(InterpPose other) {
        double xDist = other.x - x;
        double yDist = other.y - y;
        return xDist * xDist + yDist * yDist;
    }

    /**
     * Calculates the squared Euclidean distance from this pose to another pose.
     * @param other the other pose
     * @return the squared distance between the two poses
     */
    public double distSquared(Pose other) {
        double xDist = other.getX() - x;
        double yDist = other.getY() - y;
        return xDist * xDist + yDist * yDist;
    }

    /**
     * This rotates this pose by the given theta
     *
     * @param theta the angle to rotate by.
     * @param rotateHeading whether to adjust the Pose heading too.
     * @return the rotated Pose.
     */
    public InterpPose rotate(double theta, boolean rotateHeading) {
        double x = getX() * Math.cos(theta) - getY() * Math.sin(theta);
        double y = getX() * Math.sin(theta) + getY() * Math.cos(theta);
        double heading = rotateHeading ? MathFunctions.normalizeAngle(getHeading() + theta) : getHeading();

        return new InterpPose(x, y, heading, coordinateSystem);
    }

    /**
     * This mirrors this pose across x = 72 in Pedro coordinates. This will return a new Pose in Pedro coordinates.
     * @return the mirrored Pose.
     */
    public InterpPose mirror() {
        InterpPose k = getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        return new InterpPose(144 - k.getX(), k.getY(), MathFunctions.normalizeAngle(Math.PI - k.getHeading()), PedroCoordinates.INSTANCE);
    }

    /**
     * Converts this pose to the specified coordinate system.
     *
     * @param coordinateSystem the target coordinate system
     * @return the pose in the target coordinate system
     */
    public InterpPose getAsCoordinateSystem(CoordinateSystem coordinateSystem) {
        Pose converted = coordinateSystem.convertFromPedro(this.coordinateSystem.convertToPedro(getPose()));
        return new InterpPose(converted.getX(),
                converted.getY(),
                converted.getHeading(),
                converted.getCoordinateSystem());
    }

    /**
     * Converts polar coordinates to cartesian coordinates.
     *
     * @param r the radius
     * @param theta the angle in radians
     * @return an array \`[x, y]\` representing the cartesian coordinates
     */
    public static double[] polarToCartesian(double r, double theta) {
        return new double[]{r * Math.cos(theta), r * Math.sin(theta)};
    }

    /**
     * Converts cartesian coordinates to polar coordinates.
     *
     * @param x the x-coordinate
     * @param y the y-coordinate
     * @return an array \`[r, theta]\` where r is the radius and theta is the angle in radians
     */
    public static double[] cartesianToPolar(double x, double y) {
        return new double[] {Math.sqrt(x * x + y * y), MathFunctions.normalizeAngle(Math.atan2(y, x))};
    }

    /**
     * Returns a new pose with the specified heading, keeping x and y the same.
     *
     * @param heading the new heading in radians
     * @return a new pose with updated heading
     */
    public InterpPose setHeading(double heading) {
        return new InterpPose(x, y, heading);
    }

    /** Returns a new pose with the same coordinate system, keeping x, y, and heading. */
    public InterpPose copy() {
        return new InterpPose(x, y, heading, coordinateSystem);
    }

    /**
     * Returns a string representation of the pose in the format (x, y, heading in degrees).
     *
     * @return a string representation of the pose
     */
    @Override
    public String toString() {
        return "(" + getX() + ", " + getY() + ", " + Math.toDegrees(getHeading()) + ")";
    }

    /**
     * A utility method to distinguish a Pose from a FuturePose.
     * @return true, indicating that this Pose is initialized.
     */
    @Override
    public boolean initialized() {
        return true;
    }

    @Override
    public Pose getPose() {
        return new Pose(x, y, heading, coordinateSystem);
    }

    public InterpPose getInterpPose() {
        return this;
    }


    /**
     * Returns a Twist2d that maps this pose to the end pose. If c is the output of {@code a.Log(b)},
     * then {@code a.Exp(c)} would yield b.
     *
     * @param end The end pose for the transformation.
     * @return The twist that maps this to end.
     */
    public Twist log(InterpPose end) {
        final InterpPose transform = end.minus(this);
        final double dtheta = transform.getHeading();
        final double halfDtheta = dtheta / 2.0;

        final double cosMinusOne = Math.cos(transform.getHeading()) - 1;

        double halfThetaByTanOfHalfDtheta;
        if (Math.abs(cosMinusOne) < 1E-9) {
            halfThetaByTanOfHalfDtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            halfThetaByTanOfHalfDtheta = -(halfDtheta * Math.sin(transform.getHeading()) / cosMinusOne);
        }

        Translation transformTranslation = new Translation(transform.x, transform.y);
        Translation translationPart = transformTranslation
                        .rotateBy(new Rotation(halfThetaByTanOfHalfDtheta, -halfDtheta))
                        .times(Math.hypot(halfThetaByTanOfHalfDtheta, halfDtheta));

        return new Twist(translationPart.getX(), translationPart.getY(), dtheta);
    }


    /**
     * Obtain a new Pose2d from a (constant curvature) velocity.
     *
     * <p>See <a href="https://file.tavsys.net/control/controls-engineering-in-frc.pdf">Controls
     * Engineering in the FIRST Robotics Competition</a> section 10.2 "Pose exponential" for a
     * derivation.
     *
     * <p>The twist is a change in pose in the robot's coordinate frame since the previous pose
     * update. When the user runs exp() on the previous known field-relative pose with the argument
     * being the twist, the user will receive the new field-relative pose.
     *
     * <p>"Exp" represents the pose exponential, which is solving a differential equation moving the
     * pose forward in time.
     *
     * @param twist The change in pose in the robot's coordinate frame since the previous pose update.
     *     For example, if a non-holonomic robot moves forward 0.01 meters and changes angle by 0.5
     *     degrees since the previous pose update, the twist would be Twist2d(0.01, 0.0,
     *     Units.degreesToRadians(0.5)).
     * @return The new pose of the robot.
     */
    public InterpPose exp(Twist twist) {
        double dx = twist.dx;
        double dy = twist.dy;
        double dtheta = twist.dtheta;

        double sinTheta = Math.sin(dtheta);
        double cosTheta = Math.cos(dtheta);

        double s;
        double c;
        if (Math.abs(dtheta) < 1E-9) {
            s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
            c = 0.5 * dtheta;
        } else {
            s = sinTheta / dtheta;
            c = (1 - cosTheta) / dtheta;
        }

        InterpPose transform =
                new InterpPose(
                        dx * s - dy * c, dx * c + dy * s, dtheta);

        return this.plus(transform);
    }

    @Override
    public InterpPose interpolate(InterpPose endValue, double t) {
        if (t < 0) {
            return this;
        } else if (t >= 1) {
            return endValue;
        } else {
            Twist twist = this.log(endValue);
            Twist scaledTwist = new Twist(twist.dx * t, twist.dy * t, twist.dtheta * t);
            return this.exp(scaledTwist);
        }
    }

}
