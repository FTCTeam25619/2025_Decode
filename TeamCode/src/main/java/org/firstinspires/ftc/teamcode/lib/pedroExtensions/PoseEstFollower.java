package org.firstinspires.ftc.teamcode.lib.pedroExtensions;

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.PoseHistory;

import org.firstinspires.ftc.teamcode.constants.DriveConstants;

public class PoseEstFollower extends Follower {

    // This gives us an enhanced PoseEstimator reference to use in addition to the standard PoseTracker
    // reference in the base class to maintain backward compatibility.
    private final PoseEstimator poseEstimator;

    private final PoseHistory poseHistory;


    /**
     * This creates a new Follower given a HardwareMap.
     * @param constants FollowerConstants to use
     * @param localizer Localizer to use
     * @param drivetrain Drivetrain to use
     * @param pathConstraints PathConstraints to use
     */
    public PoseEstFollower(FollowerConstants constants, Localizer localizer, Drivetrain drivetrain, PathConstraints pathConstraints) {
        super(constants, localizer, drivetrain, pathConstraints);

        // Replace the standard PoseTracker object w/ enhanced PoseTracker that implements vision updates
        poseTracker = poseEstimator = new PoseEstimator(localizer);

        poseHistory = new PoseHistory(poseEstimator);

    }

    /**
     * This creates a new Follower given a HardwareMap.
     * @param constants FollowerConstants to use
     * @param localizer Localizer to use
     * @param drivetrain Drivetrain to use
     */
    public PoseEstFollower(FollowerConstants constants, Localizer localizer, Drivetrain drivetrain) {
        this(constants, localizer, drivetrain, PathConstraints.defaultConstraints);
    }

    public void addVisionUpdate(Pose visionPose, long visionTimestampNanoseconds) {
        poseEstimator.addVisionMeasurement(visionPose, visionTimestampNanoseconds);
    }

    public void addVisionUpdate(Pose visonPose,
                                long visionTimeStampNanoseconds,
                                double[] visionStdDevs) {
        poseEstimator.addVisionMeasurement(visonPose, visionTimeStampNanoseconds, visionStdDevs);
    }

    /** Calls an update to the PoseTracker, which updates the robot's current position estimate. */
    @Override
    public void updatePose() {
        super.updatePose();
        poseHistory.update();
    }

    /**
     * This returns the PoseHistory used by the Follower.
     * @return returns the PoseHistory
     */
    @Override
    public PoseHistory getPoseHistory() { return poseHistory; }


    @Override
    public Pose getPose() {
        if (DriveConstants.PoseEstimator.useVisionPoseEstimate) {
            return poseEstimator.getVisionUpdatedPose();
        }
        return super.getPose();
    }

    public Pose getUncorrectedPose() { return super.getPose(); }

}
