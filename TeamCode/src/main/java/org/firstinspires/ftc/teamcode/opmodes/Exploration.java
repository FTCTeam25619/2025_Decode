
package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.OpModes.OpModeSelection;
import org.firstinspires.ftc.teamcode.NovaPyraRobot;

@TeleOp
public class Exploration extends OpMode {
    private NovaPyraRobot robot;

    @Override
    public void init() {
        // Create FTC Dashboard if setup for shop debugging
        if (Constants.DebugModes.ENABLE_FTC_DASHBOARD) {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        }
        if (Constants.DebugModes.ENABLE_PANELS_DASHBOARD) {
            telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        }

        robot = new NovaPyraRobot(hardwareMap, gamepad1, gamepad2, telemetry, OpModeSelection.EXPLORATION);

        telemetry.addData("Robot Status", "Initializing Exploration");
    }

    @Override
    public void init_loop() {
        // Anything that needs to happen before play button
    }

    @Override
    public void start() {
        telemetry.addData("Robot Status", "Running Exploration");
        robot.initOpMode();
    }

    @Override
    public void loop() {
        robot.trackCycleTime(getRuntime());
        robot.run();
    }

    @Override
    public void stop() {
        telemetry.addData("Robot Status", "Stopped Exploration");
        robot.reset();
    }
}
