package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.constants.Alliance;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.lib.util.BlackboardHelpers;
import org.firstinspires.ftc.teamcode.lib.util.NovaPyraUtils;
import org.firstinspires.ftc.teamcode.triggers.DemoTriggers;

/**
 * Demo OpMode
 *
 * Shooting is fully manual — ShotController is bypassed for everything except intake.
 * Right trigger directly commands feedStop, shooterFeed, and intake subsystems.
 * No auto-aim, no physics solver, no presetShotMode, no RECOVERY state.
 */
@TeleOp(name = "Demo", group = "Demo")
public class DemoOpMode extends CommandOpMode {

    private NovaPyraRobot robot;
    private Alliance      alliance = Alliance.RED;

    private GamepadEx    controller1;
    private GamepadEx    controller2;
    private DemoTriggers demoTriggers;

    @Override
    public void initialize() {
        super.reset();

        robot       = new NovaPyraRobot(hardwareMap, telemetry, alliance);
        controller1 = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);

        demoTriggers = new DemoTriggers(robot, controller1, controller2);
        demoTriggers.setupGamepadButtonMappings();

        // ShotController state transitions still needed for intake side-effects
        robot.shotController.setupStateTransitionTriggers();

        // Vision started so getTargetTagPose() is never null internally
        robot.vision.startVision();
        robot.vision.setTargetingTag(alliance == Alliance.BLUE
                ? VisionConstants.AprilTags.BlueGoal
                : VisionConstants.AprilTags.RedGoal);

        Pose startingPose = BlackboardHelpers.getAutoEndPoseOrDefault(blackboard, alliance);
        robot.pedroFollower.setPose(startingPose);
        robot.pedroFollower.startTeleopDrive();

        telemetry.addLine("=== DEMO READY ===");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        // ── Drive ─────────────────────────────────────────────────────────────
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

        robot.pedroFollower.setTeleOpDrive(driveAxes[0], driveAxes[1], rotateAxis, false);
        robot.pedroFollower.update();

        // ── Vision pose update ────────────────────────────────────────────────
        if (robot.vision.isPoseValid()) {
            robot.pedroFollower.addVisionUpdate(
                    robot.vision.getPedroPose(),
                    System.nanoTime() - robot.vision.getVisionLatencyNS()
            );
        }

        // ── RPM ───────────────────────────────────────────────────────────────
        demoTriggers.updateRPM();

        // ── ShotController — intake only ──────────────────────────────────────
        // updateState() only ever sees IDLE or INTAKE in demo mode,
        // so none of the shooting/recovery logic runs.
        robot.shotController.updateState();

        // ── Hood ──────────────────────────────────────────────────────────────
        demoTriggers.updateHood();

        // ── Telemetry ─────────────────────────────────────────────────────────
        telemetry.addLine("════ DEMO MODE ════");
        telemetry.addData("RPM Selected",  String.format("%.0f", demoTriggers.getSelectedRPM()));
        telemetry.addData("RPM Actual",    String.format("%.0f", robot.shooter.getFlywheelRPM()));
        telemetry.addData("At RPM",        robot.shooter.atRPM() ? "✓" : "...");
        telemetry.addData("Hood target",   String.format("%.1f°", demoTriggers.getHoodTargetDeg()));
        telemetry.addData("Hood current",  String.format("%.1f°", demoTriggers.getHoodCurrentDeg()));
        telemetry.addData("Hood (launch)", String.format("%.1f°", robot.hood.getCurrentLaunchAngle()));
        telemetry.addLine("");
        telemetry.addLine("C1: LT=Intake  RT=Shoot  Start=ResetHeading");
        telemetry.addLine("C2: Y=Far(2900) X=Mid(2300) A=Close(1900)  RT/LT=Hood");
        telemetry.update();
    }

    @Override
    public void end() {
        robot.vision.stopVision();
    }

    @Override
    public void reset() {
        robot.reset();
    }
}