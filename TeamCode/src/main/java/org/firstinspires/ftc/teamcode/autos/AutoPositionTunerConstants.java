package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.NovaPyraRobot;

@Configurable
public class AutoPositionTunerConstants {
    public Pose startPose = new Pose(17.144, 123.233);
    public double startHeading = 143.48;

    public NovaPyraRobot robot;

    public AutoPositionTunerConstants(Follower follower, NovaPyraRobot robot) {
        this.robot = robot;
    }
}



