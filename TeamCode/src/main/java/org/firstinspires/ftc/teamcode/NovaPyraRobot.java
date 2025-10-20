package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants.OpModes.OpModeSelection;
import org.firstinspires.ftc.teamcode.constants.OpModes.OpModeSelection;
import org.firstinspires.ftc.teamcode.subsystems.Exploration;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;

/* To connect to the Control Hub device via Wi-Fi:
   - Connect Wi-Fi to the FTC-25619 network
   - In command line, run: adb connect 192.168.43.1:5555
   - To check that it is connected, run: adb devices -l
   - Check that the REV Robotics Control Hub shows up above
   - in Android Studio with a green dot (connected)
 */
public class NovaPyraRobot extends Robot {
    private final GamepadEx controller1;
    private final GamepadEx controller2;
    public static Telemetry telemetry;
    public final Odometry odometry;
    public final Exploration exploration;
    private final OpModeSelection selectedOpMode;

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
        exploration = new Exploration(hardwareMap, NovaPyraRobot.telemetry);
    }

    public void initOpMode() {
        switch (selectedOpMode) {
            case PEDRO_SAMPLE_AUTO:
            case PEDRO_SAMPLE_TELEOP:
            case EXPLORATION:
                break;
        }
    }
}
