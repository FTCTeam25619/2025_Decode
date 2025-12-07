package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.constants.OpModes;
import org.firstinspires.ftc.teamcode.constants.OpModes.OpModeSelection;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.lib.orchestration.ShotController;
import org.firstinspires.ftc.teamcode.constants.PedroConstants;
import org.firstinspires.ftc.teamcode.lib.triggers.ControllerTriggers;
import org.firstinspires.ftc.teamcode.lib.util.NovaPyraUtils;
import org.firstinspires.ftc.teamcode.subsystems.FeedStop;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.IndicatorLights;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeed;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.triggers.CompetitionTriggers;
import org.firstinspires.ftc.teamcode.triggers.TuningTriggers;

/* To connect to the Control Hub device via Wi-Fi:
   - Connect Wi-Fi to the FTC-25619 network
   - In command line, run: adb connect 192.168.43.1:5555
   - To check that it is connected, run: adb devices -l
   - Check that the REV Robotics Control Hub shows up above
   - in Android Studio with a green dot (connected)
 */
public class NovaPyraRobot extends Robot {
    public final GamepadEx controller1;
    public final GamepadEx controller2;
    public static Telemetry telemetry;
    public final Odometry odometry;
    public final ShooterFeed shooterFeed;
    public final Shooter shooter;
    public final FeedStop feedStop;
    public final Hood hood;
    public final Intake intake;
    public final Sensors sensors;
    public final IndicatorLights indicatorLights;
    public final Vision vision;
    public final ShotController shotController;
    public final Follower pedroFollower;
    private final OpModeSelection selectedOpMode;
    public final OpModes.Alliance alliance;
    private final ControllerTriggers controllerMappings;

    public NovaPyraRobot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2,
                         Telemetry telemetry, OpModeSelection opModeSelection, OpModes.Alliance alliance) {
        // OpMode selection
        selectedOpMode = opModeSelection;
        this.alliance = alliance;

        // Telemetry
        NovaPyraRobot.telemetry = telemetry;

        // Controllers
        controller1 = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);

        // Subsystems
        odometry = new Odometry(hardwareMap, NovaPyraRobot.telemetry);
        shooterFeed = new ShooterFeed(hardwareMap,NovaPyraRobot.telemetry);
        shooter = new Shooter(hardwareMap, NovaPyraRobot.telemetry);
        feedStop = new FeedStop(hardwareMap, NovaPyraRobot.telemetry);
        hood = new Hood(hardwareMap, NovaPyraRobot.telemetry);
        intake = new Intake(hardwareMap, NovaPyraRobot.telemetry);
        sensors = new Sensors(hardwareMap, NovaPyraRobot.telemetry);
        indicatorLights = new IndicatorLights(hardwareMap, NovaPyraRobot.telemetry);
        vision = new Vision(hardwareMap, odometry, NovaPyraRobot.telemetry);
        pedroFollower = PedroConstants.createFollower(hardwareMap);

        // Orchestrators
        shotController = new ShotController(this, NovaPyraRobot.telemetry);

        // ControllerMappings
        if (Constants.DebugModes.ControlInterface.TUNING_CONTROLS) {
            controllerMappings = new TuningTriggers(this);
        } else {
            controllerMappings = new CompetitionTriggers(this);
        }
    }

    public void initOpMode() {
        switch (selectedOpMode) {
            case PEDRO_SAMPLE_AUTO:
            case PEDRO_SAMPLE_TELEOP:
                break;
            case EXPLORATION:
                controllerMappings.setupGamepadButtonMappings();
                vision.startVision();
                vision.setTargetingTag(alliance == OpModes.Alliance.BLUE ?
                        VisionConstants.AprilTags.BlueGoal : VisionConstants.AprilTags.RedGoal);
                odometry.setRobotPose(new Pose2D(DistanceUnit.MM, 0.0, 0.0, AngleUnit.DEGREES, 0.0));
                break;
        }
    }

    @Override
    public void run() {
        super.run();
        switch (selectedOpMode) {
            case PEDRO_SAMPLE_AUTO:
            case PEDRO_SAMPLE_TELEOP:
                break;
            case EXPLORATION:
                double[] driveAxes = NovaPyraUtils.deadbandAndSmooth(
                        controller1.getLeftY(),
                        -controller1.getLeftX(),
                        DriveConstants.Joysticks.driveSmoothFactor,
                        DriveConstants.Joysticks.driveDeadband,
                        DriveConstants.Joysticks.driveInverseDeadband
                );

                double rotateAxis = NovaPyraUtils.deadbandAndSmoothValue(
                        -controller1.getRightX(),
                        DriveConstants.Joysticks.rotateSmoothFactor,
                        DriveConstants.Joysticks.rotateDeadband,
                        DriveConstants.Joysticks.rotateInverseDeadband
                );

                if (controller1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON) && vision.isSeeingTarget()) {
                    rotateAxis = MathUtils.clamp(
                            -DriveConstants.HeadingControl.kPHeadingCorrection * vision.angleToTarget(),
                            -DriveConstants.HeadingControl.correctionOutputClamp,
                            DriveConstants.HeadingControl.correctionOutputClamp);
                } else {
                    rotateAxis /= DriveConstants.Joysticks.rotateScalingFactor;
                }
                pedroFollower.setTeleOpDrive(driveAxes[0], driveAxes[1], rotateAxis, false);
                pedroFollower.update();
                shotController.updateState();

                telemetry.addData("X", pedroFollower.getPose().getX());
                telemetry.addData("Y", pedroFollower.getPose().getY());
                telemetry.addData("Heading", pedroFollower.getPose().getHeading());
                telemetry.update();
                break;
        }
    }

    public void endOpMode() {
        switch (selectedOpMode) {
            case PEDRO_SAMPLE_AUTO:
            case PEDRO_SAMPLE_TELEOP:
                break;
            case EXPLORATION:
                vision.stopVision();
                break;
        }
    }

    public void trackCycleTime(double newClockReading) {
        odometry.updateRevHubCycleFrequency(newClockReading);
    }
}
