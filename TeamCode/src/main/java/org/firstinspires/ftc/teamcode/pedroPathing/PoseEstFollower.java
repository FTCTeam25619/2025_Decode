package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;

import org.firstinspires.ftc.teamcode.lib.wpilibstuff.math.Matrix;
import org.firstinspires.ftc.teamcode.lib.wpilibstuff.math.numbers.N1;
import org.firstinspires.ftc.teamcode.lib.wpilibstuff.math.numbers.N3;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

public class PoseEstFollower extends Follower {

    private Vision vision;
    private final PoseEstimator poseEstimator;


    /**
     * This creates a new Follower given a HardwareMap.
     * @param constants FollowerConstants to use
     * @param localizer Localizer to use
     * @param drivetrain Drivetrain to use
     * @param pathConstraints PathConstraints to use
     */
    public PoseEstFollower(FollowerConstants constants, Localizer localizer, Drivetrain drivetrain, PathConstraints pathConstraints, Vision vision) {
        super(constants, localizer, drivetrain, pathConstraints);
        this.vision = vision;

        // Replace the standard PoseTracker object w/ enhanced PoseTracker that implements vision updates
        poseTracker = poseEstimator = new PoseEstimator(localizer, vision);
    }

    /**
     * This creates a new Follower given a HardwareMap.
     * @param constants FollowerConstants to use
     * @param localizer Localizer to use
     * @param drivetrain Drivetrain to use
     */
    public PoseEstFollower(FollowerConstants constants, Localizer localizer, Drivetrain drivetrain, Vision vision) {
        this(constants, localizer, drivetrain, PathConstraints.defaultConstraints, vision);
    }

    public void addVisionUpdate(Pose visionPose, long visionTimestampNanoseconds) {
        poseEstimator.addVisionMeasurement(visionPose, visionTimestampNanoseconds);
    }

    public void addVisionUpdate(Pose visonPose,
                                long visionTimeStampNanoseconds,
                                Matrix<N3, N1> visionStdDevs) {
        poseEstimator.addVisionMeasurement(visonPose, visionTimeStampNanoseconds, visionStdDevs);
    }

    public PoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    public Pose getVisionUpdatedPose() {
        return poseEstimator.getVisionUpdatedPose();
    }

}
