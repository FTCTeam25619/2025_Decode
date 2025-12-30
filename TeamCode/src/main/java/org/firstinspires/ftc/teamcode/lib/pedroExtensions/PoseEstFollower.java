package org.firstinspires.ftc.teamcode.lib.pedroExtensions;

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;

import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.lib.wpilibstuff.math.Matrix;
import org.firstinspires.ftc.teamcode.lib.wpilibstuff.math.numbers.N1;
import org.firstinspires.ftc.teamcode.lib.wpilibstuff.math.numbers.N3;

public class PoseEstFollower extends Follower {

    // This gives us an enhanced PoseEstimator reference to use in addition to the standard PoseTracker
    // reference in the base class to maintain backward compatibility.
    private final PoseEstimator poseEstimator;


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
                                Matrix<N3, N1> visionStdDevs) {
        poseEstimator.addVisionMeasurement(visonPose, visionTimeStampNanoseconds, visionStdDevs);
    }

    @Override
    public Pose getPose() {
        if (DriveConstants.PoseEstimator.useVisionPoseEstimate) {
            return poseEstimator.getVisionUpdatedPose();
        }
        return super.getPose();
    }

    public Pose getUncorrectedPose() { return super.getPose(); }

}
