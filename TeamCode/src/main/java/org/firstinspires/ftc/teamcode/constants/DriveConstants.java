package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.wpilibstuff.math.Matrix;
import org.firstinspires.ftc.teamcode.lib.wpilibstuff.math.VecBuilder;
import org.firstinspires.ftc.teamcode.lib.wpilibstuff.math.numbers.N1;
import org.firstinspires.ftc.teamcode.lib.wpilibstuff.math.numbers.N3;

public class DriveConstants {
    @Configurable
    public static class Joysticks {
        // TODO: Tune these values
        public static double driveSmoothFactor = 1.0;
        public static double driveDeadband = 0.1;
        public static double driveInverseDeadband = 0.05;
        public static double rotateSmoothFactor = 1.0;
        public static double rotateDeadband = 0.1;
        public static double rotateInverseDeadband = 0.05;
        public static double rotateScalingFactor = 1.6;
    }

    @Configurable
    public static class HeadingControl {
        public static double kPHeadingCorrection = 0.018;
        public static double correctionOutputClamp = 0.6;
    }

    @Configurable
    public static class PoseEstimator {
        public static final boolean useVisionPoseEstimate = true;  // If enabled apply vision updates to pose
        public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, 0.08);
        public static final Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.5, 0.5, 0.5);
    }
}
