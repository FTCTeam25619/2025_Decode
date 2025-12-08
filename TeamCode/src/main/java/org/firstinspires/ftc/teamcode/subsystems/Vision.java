package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.Devices;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.constants.OdometryConstants;

public class Vision extends SubsystemBase {
    private final Limelight3A limelightCamera;
    private final Odometry odometry;
    private final Telemetry telemetry;

    private LLResult latestResult;
    private LLResultTypes.FiducialResult latestTargetingTagResult;
    private Pose3D latestPose;
    private int targetTagID;
    private boolean seeingTarget = false;
    private double angleToTargetDeg = 0.0;
    private double distanceToTagCM = 0.0;
    private Position tagPosition = new Position();

    public Vision(HardwareMap hardwareMap, Odometry odometry, Telemetry telemetry) {
        limelightCamera = hardwareMap.get(Limelight3A.class, Devices.Vision.limelightCamera);
        limelightCamera.pipelineSwitch(0);

        this.odometry = odometry;
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        limelightCamera.updateRobotOrientation(odometry.getHeading());
        updateFromLimelight();

        if (Constants.DebugModes.Telemetry.DEBUG_VISION) {
            telemetry.addData("Vision: Tag ID", latestTargetingTagResult != null ? latestTargetingTagResult.getFiducialId() : -1);
            telemetry.addData("Vision: Target Seen?", seeingTarget);
            telemetry.addData("Vision: Target Tag", targetTagID);
            telemetry.addData("Vision: Angle to Target (deg)", angleToTargetDeg);
            telemetry.addData("Vision: Dist to Tag (cm)", distanceToTagCM);
        }
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

    /**
     * Returns true if we are at an acceptable heading toward the target.
     * If Limelight sees the tag, use LL tx (angleToTargetDeg).
     * Otherwise, fall back to odometry-based heading toward the goal point.
     */
    public boolean atHeading() {
        if (seeingTarget) {
            return Math.abs(angleToTargetDeg) <= VisionConstants.headingAngleThresholdDeg;
        } else {
            double desired = odometry.getHeadingTo(
                    OdometryConstants.Target.x,
                    OdometryConstants.Target.y
            );
            double current = odometry.getHeadingField();
            double error = Math.atan2(Math.sin(desired - current), Math.cos(desired - current));
            return Math.abs(Math.toDegrees(error)) <= VisionConstants.headingAngleThresholdDeg;
        }
    }

    public void updateFromLimelight() {
        seeingTarget = false;
        latestResult = limelightCamera.getLatestResult();
        if (latestResult.isValid()) {
            angleToTargetDeg = latestResult.getTx();
            for (LLResultTypes.FiducialResult fr : latestResult.getFiducialResults()) {
                if (fr.getFiducialId() == targetTagID) {
                    seeingTarget = true;
                    latestTargetingTagResult = fr;
                    tagPosition = fr.getTargetPoseRobotSpace().getPosition().toUnit(DistanceUnit.CM);
                    distanceToTagCM = Math.sqrt(tagPosition.x * tagPosition.x + tagPosition.z * tagPosition.z);
                    break;
                }
            }
        } else {
            angleToTargetDeg = 0.0;
            distanceToTagCM = 0.0;
        }
    }

    public Double distanceToTarget() {
        return distanceToTagCM;
    }

    public Double angleToTarget() {
        return angleToTargetDeg;
    }
}
