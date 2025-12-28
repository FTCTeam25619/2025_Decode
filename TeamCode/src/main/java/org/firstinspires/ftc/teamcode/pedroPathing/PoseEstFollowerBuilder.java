package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PoseEstFollowerBuilder extends FollowerBuilder {
    public PoseEstFollowerBuilder(FollowerConstants constants, HardwareMap hardwareMap) {
        super(constants, hardwareMap);
    }

}
