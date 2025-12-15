package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.constants.Alliance;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.Devices;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;

public class Vision extends SubsystemBase {
    private final Limelight3A limelightCamera;
    private final Odometry odometry;
    private final Telemetry telemetry;

    private LLResult latestResult;
    private LLResultTypes.FiducialResult latestTargetingTagResult;
    private Pose3D latestPose = new Pose3D(new Position(),
            new YawPitchRollAngles(AngleUnit.DEGREES, 0.0, 0.0, 0.0, 0));
    private int targetTagID;
    private boolean seeingTarget = false;
    private double angleToTargetDeg = 0.0;
    private double distanceToTagCM = 0.0;
    private Position tagPosition = new Position();

    public Vision(HardwareMap hardwareMap, Odometry odometry, Telemetry telemetry, Alliance alliance) {
        limelightCamera = hardwareMap.get(Limelight3A.class, Devices.Vision.limelightCamera);
        limelightCamera.pipelineSwitch(alliance == Alliance.BLUE ? 0 : 1);

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

    public boolean atHeading() {
        return seeingTarget && Math.abs(angleToTargetDeg) <= VisionConstants.headingAngleThresholdDeg;
    }

    public void updateFromLimelight() {
        seeingTarget = false;
        latestResult = limelightCamera.getLatestResult();
        if (latestResult.isValid()) {
            angleToTargetDeg = latestResult.getTx();

            // Pick latest vision pose from the best (first returned) tag result
            latestPose = latestResult.getBotpose_MT2();

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

    public double distanceToTarget() {
        return distanceToTagCM;
    }

    public double angleToTarget() {
        return angleToTargetDeg;
    }

    public Pose getPedroPose() {
        return new Pose(latestPose.getPosition().x, latestPose.getPosition().y, latestPose.getOrientation().getYaw());
    }
}
