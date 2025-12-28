package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;

public class PoseEstFollower extends Follower {

    public PoseEstFollower(FollowerConstants constants, Localizer localizer, Drivetrain drivetrain, PathConstraints pathConstraints) {
        super(constants, localizer, drivetrain, pathConstraints);
    }

}
