package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.NovaPyraRobot;

@Configurable
public class AutoPositionTunerConstants {
    public Pose startPose = new Pose(15.75, 114.8);
    public double startHeading = 90.0;

    public NovaPyraRobot robot;

    public AutoPositionTunerConstants(Follower follower, NovaPyraRobot robot) {
        this.robot = robot;
    }
}



