
package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.constants.ConfigConstants;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.constants.OpModes.OpModeSelection;
import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.constants.PedroConstants;
import org.firstinspires.ftc.teamcode.lib.util.NovaPyraUtils;

@TeleOp
public class Exploration extends CommandOpMode {
    private NovaPyraRobot robot;

    @Override
    public void initialize() {
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
    public void initialize_loop() {
        telemetry.addData("Robot Status", "Running Exploration");
        robot.initOpMode();

        robot.pedroFollower.startTeleopDrive();
    }

    @Override
    public void run() {
        robot.trackCycleTime(getRuntime());
        robot.run();

        double[] driveAxes = NovaPyraUtils.deadbandAndSmooth(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                DriveConstants.Joysticks.driveSmoothFactor,
                DriveConstants.Joysticks.driveDeadband,
                DriveConstants.Joysticks.driveInverseDeadband
        );

        double rotateAxis = NovaPyraUtils.deadbandAndSmoothValue(
                -gamepad1.right_stick_x,
                DriveConstants.Joysticks.rotateSmoothFactor,
                DriveConstants.Joysticks.rotateDeadband,
                DriveConstants.Joysticks.rotateInverseDeadband
        );

        robot.pedroFollower.setTeleOpDrive(driveAxes[0], driveAxes[1], rotateAxis, false);
        robot.pedroFollower.update();
        robot.shotController.updateState();

        telemetry.addData("X", robot.pedroFollower.getPose().getX());
        telemetry.addData("Y", robot.pedroFollower.getPose().getY());
        telemetry.addData("Heading", robot.pedroFollower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void end() {
        telemetry.addData("Robot Status", "Stopped Exploration");
    }

    @Override
    public void reset() {
        robot.reset();
    }
}
