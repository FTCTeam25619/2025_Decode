package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.OpModes.OpModeSelection;
import org.firstinspires.ftc.teamcode.lib.triggers.ControllerTriggers;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeed;
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
    public final Intake intake;
    private final OpModeSelection selectedOpMode;
    private final ControllerTriggers controllerMappings;

    public NovaPyraRobot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, OpModeSelection opModeSelection) {
        // OpMode selection
        selectedOpMode = opModeSelection;

        // Telemetry
        NovaPyraRobot.telemetry = telemetry;

        // Controllers
        controller1 = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);

        // Subsystems
        odometry = new Odometry(hardwareMap, NovaPyraRobot.telemetry);
        shooterFeed = new ShooterFeed(hardwareMap,NovaPyraRobot.telemetry);
        shooter = new Shooter(hardwareMap, NovaPyraRobot.telemetry);
        intake = new Intake(hardwareMap, NovaPyraRobot.telemetry);

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
                odometry.setRobotPose(new Pose2D(DistanceUnit.MM, 0.0, 0.0, AngleUnit.DEGREES, 0.0));
                break;
        }
    }

    public void trackCycleTime(double newClockReading) {
        odometry.updateRevHubCycleFrequency(newClockReading);
    }
}
