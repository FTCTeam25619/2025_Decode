package org.firstinspires.ftc.teamcode.lib.pedroExtensions;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.lib.wpilibstuff.math.interpolation.TimeInterpolatableBuffer;

import java.util.NavigableMap;
import java.util.Optional;
import java.util.TreeMap;

public class PoseEstimator extends PoseTracker {

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

    private final double[] m_q = new double[3];
    private final double[] m_visionK = new double[3];

    private static final double kBufferDuration = 1.5;
    // Maps timestamps to odometry-only pose estimates
    private final TimeInterpolatableBuffer<InterpPose> m_odometryPoseBuffer =
            TimeInterpolatableBuffer.createBuffer(kBufferDuration);
    // Maps timestamps to vision updates
    // Always contains one entry before the oldest entry in m_odometryPoseBuffer, unless there have
    // been no vision measurements after the last reset
    private final NavigableMap<Double, VisionUpdate> m_visionUpdates = new TreeMap<>();

    private Pose m_poseEstimate;    // The vision updated pose -- with vision compensation applied to the raw pose + offsets


    /**
     * Creates a new PoseTracker from a Localizer.
     *
     * @param localizer the Localizer
     */
    public PoseEstimator(Localizer localizer) {
        super(localizer);
        this.localizer = localizer;

//        This is commented out as its function is performed by the superclass constructor
//        try {
//            localizer.resetIMU();
//        } catch (InterruptedException ignored) {
//            System.out.println("PoseTracker: resetIMU() interrupted");
//        }
        setOdometryMeasurementStdDevs(DriveConstants.PoseEstimator.stateStdDevs);
        setVisionMeasurementStdDevs(DriveConstants.PoseEstimator.visionStdDevs);
        updatePoseEstimate();
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
        localizer.update();    // Don't run this as the superclass is updating the hardware for us

        m_odometryPoseBuffer.addSample(System.nanoTime() / 1.0E9, new InterpPose(applyOffset(getRawPose())));
        updatePoseEstimate();
    }


    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * <p>This method can be called as infrequently as you want, as long as you are calling {@link
     * PoseEstimator#update} every loop.
     *
     * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
     * recommend only adding vision measurements that are already within one meter or so of the
     * current pose estimate.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampNanoseconds The timestamp of the vision measurement in seconds. Note that if you
     *     don't use your own time source by calling {@link
     *     PoseEstimator#addVisionMeasurement(Pose, double)} then you must use a timestamp with
     *     an epoch since FPGA startup (i.e., the epoch of this timestamp is the same epoch as
     *     System.nanotime().) This means that you should use System.nanoTime() as your time source or sync the epochs.
     */
    public void addVisionMeasurement(Pose visionRobotPoseMeters, double timestampNanoseconds) {
        double timestampSeconds = timestampNanoseconds / 1.0E9;

        if (visionRobotPoseMeters.getX() == 72.0 && visionRobotPoseMeters.getY() == 72.0) {
            // Default null pose for Limelight
            return;
        }

        if (visionRobotPoseMeters.getX() <= 0.0 || visionRobotPoseMeters.getX() >= 144.0 ||
                visionRobotPoseMeters.getY() <= 0.0 || visionRobotPoseMeters.getY() >= 144.0) {
            // Pose is outside field perimeter
            return;
        }

        if (Math.toDegrees(localizer.getVelocity().getHeading()) >= DriveConstants.PoseEstimator.maxSlewRateDPS) {
            // Slew rate is too large
            return;
        }

        InterpPose visionInterpPose = new InterpPose(visionRobotPoseMeters);
        // Step 0: If this measurement is old enough to be outside the pose buffer's timespan, skip.

        if (m_odometryPoseBuffer.getInternalBuffer().isEmpty()
                || m_odometryPoseBuffer.getInternalBuffer().lastKey() - kBufferDuration
                > timestampSeconds) {
            return;
        }

        // Step 1: Clean up any old entries
        cleanUpVisionUpdates();

        // Step 2: Get the pose measured by odometry at the moment the vision measurement was made.
        InterpPose odometrySample = m_odometryPoseBuffer.getSample(timestampSeconds).get();

        if (odometrySample == null)//.isEmpty()) {
        {
            return;
        }

        // Step 3: Get the vision-compensated pose estimate at the moment the vision measurement was
        // made.
        InterpPose visionSample = sampleAt(timestampSeconds).get();

        if (visionSample == null) //.isEmpty()) {
        {
            return;
        }

        // Step 4: Measure the twist between the old pose estimate and the vision pose.
        Twist twist = visionSample.log(visionInterpPose);

        // Step 5: We should not trust the twist entirely, so instead we scale this twist by a Kalman
        // gain matrix representing how much we trust vision measurements compared to our current pose.
        Twist scaledTwist = new Twist (
                twist.dx * m_visionK[0],
                twist.dy * m_visionK[1],
                twist.dtheta * m_visionK[2]);

        // Step 7: Calculate and record the vision update.
        InterpPose expPose = visionSample.exp(scaledTwist);
        VisionUpdate visionUpdate = new VisionUpdate(visionSample.exp(scaledTwist), odometrySample);
        m_visionUpdates.put(timestampSeconds, visionUpdate);

        // Step 8: Remove later vision measurements. (Matches previous behavior)
        m_visionUpdates.tailMap(timestampSeconds, false).entrySet().clear();

        // Step 9: Update latest pose estimate. Since we cleared all updates after this vision update,
        // it's guaranteed to be the latest vision update.
        m_poseEstimate = visionUpdate.compensate(getUncorrectedPose());
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * <p>This method can be called as infrequently as you want, as long as you are calling {@link
     * PoseEstimator#update} every loop.
     *
     * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
     * recommend only adding vision measurements that are already within one meter or so of the
     * current pose estimate.
     *
     * <p>Note that the vision measurement standard deviations passed into this method will continue
     * to apply to future measurements until a subsequent call to {@link
     * PoseEstimator#setVisionMeasurementStdDevs(double[])} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampNanoseconds The timestamp of the vision measurement in seconds. Note that if you
     *     don't use your own time source by calling addVisionMeasurement, then you must use a
     *     timestamp with an epoch since FPGA startup (i.e., the epoch of this timestamp is the same
     *     epoch as System.nanoTime()). This means that you
     *     should use System.nanoTime() as your time source in
     *     this case.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
     *     in meters, y position in meters, and heading in radians). Increase these numbers to trust
     *     the vision pose measurement less.
     */
    public void addVisionMeasurement(
            Pose visionRobotPoseMeters,
            double timestampNanoseconds,
            double[] visionMeasurementStdDevs) {
        setVisionMeasurementStdDevs(visionMeasurementStdDevs);
        addVisionMeasurement(visionRobotPoseMeters, timestampNanoseconds);
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
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
        updatePoseEstimate();
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
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
        updatePoseEstimate();
    }

    /**
     * This sets the offset for only the x position.
     *
     * @param offset This sets the offset.
     */
    @Override
    public void setXOffset(double offset) {
        xOffset = offset;
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
        updatePoseEstimate();
    }

    /**
     * This sets the offset for only the y position.
     *
     * @param offset This sets the offset.
     */
    @Override
    public void setYOffset(double offset) {
        yOffset = offset;
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
        updatePoseEstimate();
    }

    /**
     * This sets the offset for only the heading.
     *
     * @param offset This sets the offset.
     */
    @Override
    public void setHeadingOffset(double offset) {
        headingOffset = offset;
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
        updatePoseEstimate();
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
    private InterpPose applyOffset(InterpPose pose) {
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
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
        updatePoseEstimate();
    }

    /**
     * This returns the current pose, with offsets applied. If this is called multiple times in
     * a single update, the current pose is cached so that subsequent calls don't have to repeat
     * localizer calls or calculations.
     *
     * @return returns the current pose.
     */
    public Pose getUncorrectedPose() {
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
     * Return the Estimated Pose including vision compensation
     *
     */

    public Pose getVisionUpdatedPose() {
        updatePoseEstimate();
        return m_poseEstimate;
    }

    @Override
    public Pose getPose() {
        if (DriveConstants.PoseEstimator.useVisionPoseEstimate) {
            return getVisionUpdatedPose();
        }
        return getUncorrectedPose();
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
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
        updatePoseEstimate();
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
        Pose returnPose = getUncorrectedPose();
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
            localizer.setPose(new Pose(getUncorrectedPose().getX(), getUncorrectedPose().getY(), getNormalizedIMUHeading() + startingPose.getHeading()));
        }
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
        updatePoseEstimate();
    }

    /**
     * This resets the heading of the robot to the IMU's heading, using offsets instead of Road
     * Runner's pose reset. This way, it's faster, but this can be wiped with the resetOffsets()
     * method.
     */
    @Override
    public void resetHeadingToIMUWithOffsets() {
        if (Double.isNaN(getIMUHeadingEstimate()) || Double.isInfinite(getIMUHeadingEstimate())) {
            setCurrentPoseWithOffset(new Pose(getUncorrectedPose().getX(), getUncorrectedPose().getY(), getNormalizedIMUHeading() + startingPose.getHeading()));
        }
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
        updatePoseEstimate();
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
        m_odometryPoseBuffer.clear();
        m_visionUpdates.clear();
    }

    @Override
    public String debugString() {
        return "PoseTracker{" +
                "currentPose=" + getUncorrectedPose() +
                ", previousPose=" + getPreviousPose() +
                ", currentVelocity=" + getVelocity() +
                ", previousVelocity=" + previousVelocity +
                ", currentAcceleration=" + getAcceleration() +
                ", xOffset=" + getXOffset() +
                ", yOffset=" + getYOffset() +
                ", headingOffset=" + getHeadingOffset() +
                '}';
    }


    /**
     * Sets the pose estimator's trust of odometry measurements.
     *
     * @param odometryMeasurementStdDevs Standard deviations of the odometry measurements. Increase these
     *     numbers to trust global measurements from odometry less. This matrix is in the form [x, y,
     *     theta]ᵀ, with units in meters and radians.
     */
    public final void setOdometryMeasurementStdDevs(double[] odometryMeasurementStdDevs) {
        for (int i = 0; i < 3; ++i) {
            m_q[i] = odometryMeasurementStdDevs[i] * odometryMeasurementStdDevs[i];
        }

    }

    /**
     * Sets the pose estimator's trust of global measurements. This might be used to change trust in
     * vision measurements after the autonomous period, or to change trust as distance to a vision
     * target increases.
     *
     * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these
     *     numbers to trust global measurements from vision less. This matrix is in the form [x, y,
     *     theta]ᵀ, with units in meters and radians.
     */
    public final void setVisionMeasurementStdDevs(double[] visionMeasurementStdDevs) {
        double[] r = new double[3];
        for (int i = 0; i < 3; ++i) {
            r[i] = visionMeasurementStdDevs[i] * visionMeasurementStdDevs[i];
        }

        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md.
        for (int row = 0; row < 3; ++row) {
            if (m_q[row] == 0.0) {
                m_visionK[row] = 0.0;
            } else {
                m_visionK[row] = m_q[row] / (m_q[row] + Math.sqrt(m_q[row] * r[row]));
            }
        }
    }

    /**
     * Return the pose at a given timestamp, if the buffer is not empty.
     *
     * @param timestampSeconds The pose's timestamp in seconds.
     * @return The pose at the given timestamp (or Optional.empty() if the buffer is empty).
     */
    private Optional<InterpPose> sampleAt(double timestampSeconds) {
        // Step 0: If there are no odometry updates to sample, skip.
        if (m_odometryPoseBuffer.getInternalBuffer().isEmpty()) {
            return Optional.empty();
        }

        // Step 1: Make sure timestamp matches the sample from the odometry pose buffer. (When sampling,
        // the buffer will always use a timestamp between the first and last timestamps)
        double oldestOdometryTimestamp = m_odometryPoseBuffer.getInternalBuffer().firstKey();
        double newestOdometryTimestamp = m_odometryPoseBuffer.getInternalBuffer().lastKey();
        timestampSeconds =
                MathUtils.clamp(timestampSeconds, oldestOdometryTimestamp, newestOdometryTimestamp);

        // Step 2: If there are no applicable vision updates, use the odometry-only information.
        if (m_visionUpdates.isEmpty() || timestampSeconds < m_visionUpdates.firstKey()) {
            return m_odometryPoseBuffer.getSample(timestampSeconds);
        }

        // Step 3: Get the latest vision update from before or at the timestamp to sample at.
        double floorTimestamp = m_visionUpdates.floorKey(timestampSeconds);
        VisionUpdate visionUpdate = m_visionUpdates.get(floorTimestamp);

        // Step 4: Get the pose measured by odometry at the time of the sample.
        Optional<InterpPose> odometryEstimate = m_odometryPoseBuffer.getSample(timestampSeconds);

        // Step 5: Apply the vision compensation to the odometry pose.
        return odometryEstimate.map(odometryPose -> visionUpdate.compensate(odometryPose));
    }


    /** Removes stale vision updates that won't affect sampling. */
    private void cleanUpVisionUpdates() {
        // Step 0: If there are no odometry samples, skip.
        if (m_odometryPoseBuffer.getInternalBuffer().isEmpty()) {
            return;
        }

        // Step 1: Find the oldest timestamp that needs a vision update.
        double oldestOdometryTimestamp = m_odometryPoseBuffer.getInternalBuffer().firstKey();

        // Step 2: If there are no vision updates before that timestamp, skip.
        if (m_visionUpdates.isEmpty() || oldestOdometryTimestamp < m_visionUpdates.firstKey()) {
            return;
        }

        // Step 3: Find the newest vision update timestamp before or at the oldest timestamp.
        double newestNeededVisionUpdateTimestamp = m_visionUpdates.floorKey(oldestOdometryTimestamp);

        // Step 4: Remove all entries strictly before the newest timestamp we need.
        m_visionUpdates.headMap(newestNeededVisionUpdateTimestamp, false).clear();
    }

    private void updatePoseEstimate() {
        if (!DriveConstants.PoseEstimator.useVisionPoseEstimate || m_visionUpdates.isEmpty()) {
            m_poseEstimate = getUncorrectedPose();
        } else {
            VisionUpdate visionUpdate = m_visionUpdates.get(m_visionUpdates.lastKey());
            m_poseEstimate = visionUpdate.compensate(getUncorrectedPose());
        }
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

        public Pose compensate(Pose pose) {
            Pose delta = pose.minus(this.odometryPose.getPose());
            return this.visionPose.getPose().plus(delta);
        }
    }



}