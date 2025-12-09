
package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.OpModes;
import org.firstinspires.ftc.teamcode.constants.OpModes.OpModeSelection;
import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.triggers.RobotTriggers;

public class Exploration extends CommandOpMode {
    private NovaPyraRobot robot;
    public OpModes.Alliance alliance;
    private RobotTriggers robotTriggers;

    public Exploration(OpModes.Alliance allianceSetting) {
        super();
        alliance = allianceSetting;
    }

    @Override
    public void initialize() {
        // Setup Dashboard if setup for shop debugging
        if (Constants.DebugModes.ENABLE_PANELS_DASHBOARD) {
            telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        }

        robot = new NovaPyraRobot(hardwareMap, gamepad1, gamepad2, telemetry,
                OpModeSelection.EXPLORATION, alliance);
        robotTriggers = new RobotTriggers(robot);

        telemetry.addData("Robot Status", "Initializing Exploration " + alliance.name());
        robotTriggers.setupNonControllerTriggers();
    }

    @Override
    public void initialize_loop() {
        telemetry.addData("Robot Status", "Running Exploration " + alliance.name());
        robot.initOpMode();

        robot.pedroFollower.startTeleopDrive();
    }

    @Override
    public void run() {
        robot.trackCycleTime(getRuntime());
        robot.run();
    }

    @Override
    public void end() {
        telemetry.addData("Robot Status", "Stopped Exploration " + alliance.name());
        robot.endOpMode();
    }

    @Override
    public void reset() {
        robot.reset();
    }
}
