package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.lib.wpilibstuff.math.Matrix;
import org.firstinspires.ftc.teamcode.lib.wpilibstuff.math.Nat;
import org.firstinspires.ftc.teamcode.lib.wpilibstuff.math.interpolation.TimeInterpolatableBuffer;
import org.firstinspires.ftc.teamcode.lib.wpilibstuff.math.numbers.N1;
import org.firstinspires.ftc.teamcode.lib.wpilibstuff.math.numbers.N3;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.util.NavigableMap;
import java.util.TreeMap;

public class PoseEstimator extends PoseTracker {
    private Vision vision;

    /**
     * Have to shadow all the instance variables since they are private and we can't access
     * We just override all methods and use our subclass copy of variables and reimplement everythign ehre.
     *
     * We do this so we don't have to completely re-implemnt the Follower class -- Follower can call PoseEstimator
     * methods as a subclass of PoseTracker as normal -- and only need to add those vision specific calls to follower.
     */
    private final Localizer localizer;

    private InterpPose startingPose = new InterpPose(0,0,0);

    private InterpPose currentPose = startingPose.copy();

    private InterpPose previousPose = startingPose.copy();

    private InterpPose currentVelocity = new InterpPose();

    private Vector previousVelocity = new Vector();

    private Vector currentAcceleration = new Vector();

    private double xOffset = 0;
    private double yOffset = 0;
    private double headingOffset = 0;

    private long previousPoseTime;
    private long currentPoseTime;

    private final Matrix<N3, N1> m_q = new Matrix<>(Nat.N3(), Nat.N1());
    private final Matrix<N3, N3> m_visionK = new Matrix<>(Nat.N3(), Nat.N3());

    private static final double kBufferDuration = 1.5;
    // Maps timestamps to odometry-only pose estimates
    private final TimeInterpolatableBuffer<InterpPose> m_odometryPoseBuffer =
            TimeInterpolatableBuffer.createBuffer(kBufferDuration);
    // Maps timestamps to vision updates
    // Always contains one entry before the oldest entry in m_odometryPoseBuffer, unless there have
    // been no vision measurements after the last reset
    private final NavigableMap<Double, VisionUpdate> m_visionUpdates = new TreeMap<>();

    private InterpPose m_poseEstimate;



    /**
     * Creates a new PoseTracker from a Localizer.
     *
     * @param localizer the Localizer
     */
    public PoseEstimator(Localizer localizer, Vision vision) {
        super(localizer);
        this.localizer = localizer;
        this.vision = vision;

//        This is commented out as its function is performed by the superclass constructor
//        try {
//            localizer.resetIMU();
//        } catch (InterruptedException ignored) {
//            System.out.println("PoseTracker: resetIMU() interrupted");
//        }
    }


    /**
     * This updates the robot's pose, as well as updating the previous pose, velocity, and
     * acceleration. The cache for the current pose, velocity, and acceleration is cleared, and
     * the time stamps are updated as well.
     */
    @Override
    public void update() {
        previousVelocity = getVelocity();
        previousPose = new InterpPose(applyOffset(getRawPose()));
        currentPose = null;
        currentVelocity = null;
        currentAcceleration = null;
        previousPoseTime = currentPoseTime;
        currentPoseTime = System.nanoTime();
        localizer.update();
    }

    /**
     * This sets the starting pose. Do not run this after moving at all.
     *
     * @param set the Pose to set the starting pose to.
     */
    @Override
    public void setStartingPose(Pose set) {

        startingPose = new InterpPose(currentPose.getX(),
                currentPose.getY(),
                currentPose.getHeading(),
                currentPose.getCoordinateSystem());
        previousPose = startingPose;
        previousPoseTime = System.nanoTime();
        currentPoseTime = System.nanoTime();
        localizer.setStartPose(set);
    }

    /**
     * This sets the current pose, using offsets. Think of using offsets as setting trim in an
     * aircraft. This can be reset as well, so beware of using the resetOffset() method.
     *
     *
     * @param set The pose to set the current pose to.
     */
    @Override
    public void setCurrentPoseWithOffset(Pose set) {
        Pose currentPose = getRawPose();
        setXOffset(set.getX() - currentPose.getX());
        setYOffset(set.getY() - currentPose.getY());
        setHeadingOffset(MathFunctions.getTurnDirection(currentPose.getHeading(), set.getHeading()) * MathFunctions.getSmallestAngleDifference(currentPose.getHeading(), set.getHeading()));
    }

    /**
     * This sets the offset for only the x position.
     *
     * @param offset This sets the offset.
     */
    @Override
    public void setXOffset(double offset) {
        xOffset = offset;
    }

    /**
     * This sets the offset for only the y position.
     *
     * @param offset This sets the offset.
     */
    @Override
    public void setYOffset(double offset) {
        yOffset = offset;
    }

    /**
     * This sets the offset for only the heading.
     *
     * @param offset This sets the offset.
     */
    @Override
    public void setHeadingOffset(double offset) {
        headingOffset = offset;
    }

    /**
     * This returns the x offset.
     *
     * @return returns the x offset.
     */
    @Override
    public double getXOffset() {
        return xOffset;
    }

    /**
     * This returns the y offset.
     *
     * @return returns the y offset.
     */
    @Override
    public double getYOffset() {
        return yOffset;
    }

    /**
     * This returns the heading offset.
     *
     * @return returns the heading offset.
     */
    @Override
    public double getHeadingOffset() {
        return headingOffset;
    }

    /**
     * This applies the offset to a specified Pose.
     *
     * @param pose The pose to be offset.
     * @return This returns a new Pose with the offset applied.
     */
    public InterpPose applyOffset(InterpPose pose) {
        return new InterpPose(pose.getX()+xOffset, pose.getY()+yOffset, pose.getHeading()+headingOffset);
    }

    /**
     * This resets all offsets set to the PoseTracker. If you have reset your pose using the
     * setCurrentPoseUsingOffset(Pose2d set) method, then your pose will be returned to what the
     * PoseTracker thinks your pose would be, not the pose you reset to.
     */
    @Override
    public void resetOffset() {
        setXOffset(0);
        setYOffset(0);
        setHeadingOffset(0);
    }

    /**
     * This returns the current pose, with offsets applied. If this is called multiple times in
     * a single update, the current pose is cached so that subsequent calls don't have to repeat
     * localizer calls or calculations.
     *
     * @return returns the current pose.
     */
    @Override
    public Pose getPose() {
        if (currentPose == null) {
            currentPose = new InterpPose(
                    localizer.getPose().getX(),
                    localizer.getPose().getY(),
                    localizer.getPose().getHeading(),
                    localizer.getPose().getCoordinateSystem());
        }
        return applyOffset(currentPose).getPose();
    }

    /**
     * This returns the current raw pose, without any offsets applied. If this is called multiple times in
     * a single update, the current pose is cached so that subsequent calls don't have to repeat
     * localizer calls or calculations.
     *
     * @return returns the raw pose.
     */
    @Override
    public Pose getRawPose() {
        if (currentPose == null) {
            currentPose = new InterpPose(
                    localizer.getPose().getX(),
                    localizer.getPose().getY(),
                    localizer.getPose().getHeading(),
                    localizer.getPose().getCoordinateSystem());
        }
        return currentPose.getPose();
    }

    /**
     * This sets the current pose without using resettable offsets.
     *
     * @param set the pose to set the current pose to.
     */
    @Override
    public void setPose(Pose set) {
        resetOffset();
        localizer.setPose(set);
    }

    /**
     * Returns the robot's pose from the previous update.
     *
     * @return returns the robot's previous pose.
     */
    @Override
    public Pose getPreviousPose() {
        return previousPose.getPose();
    }

    /**
     * Returns the robot's change in pose from the previous update.
     *
     * @return returns the robot's delta pose.
     */
    @Override
    public Pose getDeltaPose() {
        Pose returnPose = getPose();
        return returnPose.minus(previousPose.getPose());
    }

    /**
     * This returns the velocity of the robot as a Vector. If this is called multiple times in
     * a single update, the velocity Vector is cached so that subsequent calls don't have to repeat
     * localizer calls or calculations.
     *
     * @return returns the velocity of the robot.
     */
    @Override
    public Vector getVelocity() {
        if (currentVelocity == null) currentVelocity = new InterpPose(localizer.getVelocity());
        return currentVelocity.getAsVector();
    }

    /**
     * This returns the angular velocity of the robot as a double.
     *
     * @return returns the angular velocity of the robot.
     */
    @Override
    public double getAngularVelocity() {
        if (currentVelocity == null) currentVelocity = new InterpPose(localizer.getVelocity());
        return currentVelocity.getHeading();
    }

    /**
     * This returns the acceleration of the robot as a Vector. If this is called multiple times in
     * a single update, the acceleration Vector is cached so that subsequent calls don't have to
     * repeat localizer calls or calculations.
     *
     * @return returns the acceleration of the robot.
     */
    @Override
    public Vector getAcceleration() {
        if (currentAcceleration == null) {
            currentAcceleration = getVelocity().minus(previousVelocity);
            currentAcceleration.setMagnitude(currentAcceleration.getMagnitude() / ((currentPoseTime - previousPoseTime) / Math.pow(10.0, 9)));
        }
        return currentAcceleration.copy();
    }

    /**
     * This resets the heading of the robot to the IMU's heading, using Road Runner's pose reset.
     */
    @Override
    public void resetHeadingToIMU() {
        if (Double.isNaN(getIMUHeadingEstimate()) || Double.isInfinite(getIMUHeadingEstimate())) {
            localizer.setPose(new Pose(getPose().getX(), getPose().getY(), getNormalizedIMUHeading() + startingPose.getHeading()));
        }
    }

    /**
     * This resets the heading of the robot to the IMU's heading, using offsets instead of Road
     * Runner's pose reset. This way, it's faster, but this can be wiped with the resetOffsets()
     * method.
     */
    @Override
    public void resetHeadingToIMUWithOffsets() {
        if (Double.isNaN(getIMUHeadingEstimate()) || Double.isInfinite(getIMUHeadingEstimate())) {
            setCurrentPoseWithOffset(new Pose(getPose().getX(), getPose().getY(), getNormalizedIMUHeading() + startingPose.getHeading()));
        }
    }

    /**
     * This returns the IMU heading normalized to be between [0, 2 PI] radians.
     *
     * @return returns the normalized IMU heading.
     */
    @Override
    public double getNormalizedIMUHeading() {
        if (Double.isNaN(getIMUHeadingEstimate()) || Double.isInfinite(getIMUHeadingEstimate())) {
            return MathFunctions.normalizeAngle(-getIMUHeadingEstimate());
        }
        return 0;
    }

    /**
     * This returns the total number of radians the robot has turned.
     *
     * @return the total heading.
     */
    @Override
    public double getTotalHeading() {
        return localizer.getTotalHeading();
    }

    /**
     * This returns the Localizer.
     *
     * @return the Localizer
     */
    @Override
    public Localizer getLocalizer() {
        return localizer;
    }

    /**
     * This returns the IMU heading.
     *
     * @return the IMU heading
     */
    @Override
    public double getIMUHeadingEstimate() {
        return localizer.getIMUHeading();
    }

    /**
     * This resets the IMU of the localizer.
     */
    @Override
    public void resetIMU() throws InterruptedException {
        localizer.resetIMU();
    }

    @Override
    public String debugString() {
        return "PoseTracker{" +
                "currentPose=" + getPose() +
                ", previousPose=" + getPreviousPose() +
                ", currentVelocity=" + getVelocity() +
                ", previousVelocity=" + previousVelocity +
                ", currentAcceleration=" + getAcceleration() +
                ", xOffset=" + getXOffset() +
                ", yOffset=" + getYOffset() +
                ", headingOffset=" + getHeadingOffset() +
                '}';
    }

    private static final class VisionUpdate {
        // The vision-compensated pose estimate.
        private final InterpPose visionPose;

        // The pose estimated based solely on odometry.
        private final InterpPose odometryPose;

        /**
         * Constructs a vision update record with the specified parameters.
         *
         * @param visionPose The vision-compensated pose estimate.
         * @param odometryPose The pose estimate based solely on odometry.
         */
        private VisionUpdate(InterpPose visionPose, InterpPose odometryPose) {
            this.visionPose = visionPose;
            this.odometryPose = odometryPose;
        }

        /**
         * Returns the vision-compensated version of the pose. Specifically, changes the pose from being
         * relative to this record's odometry pose to being relative to this record's vision pose.
         *
         * @param pose The pose to compensate.
         * @return The compensated pose.
         */
        public InterpPose compensate(InterpPose pose) {
            InterpPose delta = pose.minus(this.odometryPose);
            return this.visionPose.plus(delta);
        }
    }



}

