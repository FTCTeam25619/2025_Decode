package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.constants.Devices;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;

public class Vision extends SubsystemBase {
    private Limelight3A limelightCamera;

    private LLResult latestResult;
    private LLResultTypes.FiducialResult latestTargetingTagResult;
    private Pose3D latestPose;
    private int targetTagID;
    private boolean seeingTarget = false;

    public Vision(HardwareMap hardwareMap, Telemetry telemetry) {
        limelightCamera = hardwareMap.get(Limelight3A.class, Devices.Vision.limelightCamera);
    }

    public void startVision() {
        limelightCamera.start();
    }

    public void stopVision() {
        limelightCamera.stop();
    }

    public void setTargetingTag(VisionConstants.AprilTags tag) {
        targetTagID = tag.id;
    }

    public boolean isSeeingTarget() {
        return seeingTarget;
    }

    public void updateFromLimelight() {
        seeingTarget = false;
        latestResult = limelightCamera.getLatestResult();
        if (latestResult.isValid()) {
            latestPose = latestResult.getBotpose_MT2();
            for (LLResultTypes.FiducialResult fr : latestResult.getFiducialResults()) {
                if (fr.getFiducialId() == targetTagID) {
                    seeingTarget = true;
                    latestTargetingTagResult = fr;
                    break;
                }
            }
        }
    }

    public Pose3D getBotPose() {
        if (seeingTarget) {
            return latestPose;
        }
        return null;
    }

    public Double distanceToTarget() {
        if (seeingTarget) {
            // Calculate distance from target result
        }
        return null;
    }

    public Double angleToTarget() {
        if (seeingTarget) {
            // Calculate angle from target result
            // Maybe use crosshairs?
        }
        return null;
    }
}
