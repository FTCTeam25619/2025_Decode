package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;

import org.firstinspires.ftc.teamcode.subsystems.Vision;

public class PoseEstFollower extends Follower {

    private Vision vision;


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
        poseTracker = new PoseEstimator(localizer, vision);
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

    public PoseEstFollower(FollowerConstants constants, Localizer localizer, Drivetrain drivetrain, PathConstraints pathConstraints) {
        super(constants, localizer, drivetrain, pathConstraints);
    }
}
