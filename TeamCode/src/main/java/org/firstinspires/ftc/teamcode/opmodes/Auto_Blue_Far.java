package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.NovaPyraRobot;
import org.firstinspires.ftc.teamcode.constants.OpModes;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.lib.orchestration.ShotCycleState;

@Autonomous(name = "NP – Left Triangle 3 Preloads", group = "NovaPyra")
public class Auto_Blue_Far extends CommandOpMode {

    // ---------------- FIELD / GEOMETRY (INCHES) ----------------

    // 12 ft x 12 ft field => 144 in x 144 in.
    // Origin (0,0) = bottom-left. Left goal is at top-left.
    private static final double GOAL_X_IN = 0.0;
    private static final double GOAL_Y_IN = 144.0;

    // ---- START POSE ON LEFT SIDE OF BOTTOM TRIANGLE (TUNE THESE) ----
    // Measure from bottom-left corner of the field, in inches.
    private static final double START_X_IN = 32.0;   // TODO: tune on field
    private static final double START_Y_IN = 18.0;   // TODO: tune on field

    // Starting heading: roughly upfield.
    private static final double START_HEADING_RAD = Math.toRadians(90.0);

    // How far to drive straight “upfield” off the triangle after shooting (inches)
    private static final double DRIVE_OFF_DISTANCE_IN = 16.0; // tune to be clearly off triangle

    // ---------------- ROBOT / PEDRO / TELEMETRY ----------------

    private NovaPyraRobot robot;
    private Follower follower;
    private TelemetryData telemetryData;

    // Pedro poses (inches, radians)
    private Pose startPose;
    private Pose aimPose;
    private Pose offTrianglePose;

    // Pedro paths
    private PathChain rotateToGoal;
    private PathChain driveOffTriangle;

    // ---------------- PATH SETUP ----------------

    private void buildPosesAndPaths() {
        // Heading from start to goal in field coords (inches)
        double headingToGoal = Math.atan2(
                GOAL_Y_IN - START_Y_IN,
                GOAL_X_IN - START_X_IN
        );

        // Robot start on the red X
        startPose = new Pose(START_X_IN, START_Y_IN, START_HEADING_RAD);

        // Aim pose: same spot, rotated toward the goal.
        // Add a tiny Y delta so the BezierLine isn’t zero-length.
        aimPose = new Pose(START_X_IN, START_Y_IN + 6.0, headingToGoal);

        // Drive-off pose: forward off the triangle, same heading.
        offTrianglePose = new Pose(
                START_X_IN,
                START_Y_IN + DRIVE_OFF_DISTANCE_IN,
                headingToGoal
        );

        // Path 1: rotate (plus tiny translation) to face the goal
        rotateToGoal = follower.pathBuilder()
                .addPath(new BezierLine(startPose, aimPose))
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        aimPose.getHeading())
                .build();

        // Path 2: drive off the triangle
        driveOffTriangle = follower.pathBuilder()
                .addPath(new BezierLine(aimPose, offTrianglePose))
                .setLinearHeadingInterpolation(
                        aimPose.getHeading(),
                        offTrianglePose.getHeading())
                .build();
    }

    // ---------------- INIT ----------------

    @Override
    public void initialize() {
        super.reset();

        telemetryData = new TelemetryData(telemetry);

        // Create robot; we’ll treat this as a Pedro-style auto opmode
        robot = new NovaPyraRobot(
                hardwareMap,
                gamepad1,
                gamepad2,
                telemetry,
                OpModes.OpModeSelection.PEDRO_SAMPLE_AUTO,
                OpModes.Alliance.RED   // starting on red side
        );

        follower = robot.pedroFollower;

        // Pedro pose in INCHES
        follower.setStartingPose(new Pose(
                START_X_IN,
                START_Y_IN,
                START_HEADING_RAD
        ));

        // Sync odometry to same pose (Pinpoint uses mm)
        double startXmm = START_X_IN * 25.4;
        double startYmm = START_Y_IN * 25.4;
        robot.odometry.setRobotPose(new Pose2D(
                DistanceUnit.MM,
                startXmm,
                startYmm,
                AngleUnit.RADIANS,
                START_HEADING_RAD
        ));

        // Vision: start and set the left goal tag
        robot.vision.startVision();
        robot.vision.setTargetingTag(VisionConstants.AprilTags.RedGoal);

        // ShotController: make sure transition triggers are wired
        robot.shotController.setupStateTransitionTriggers();

        // Build Pedro paths for this routine (inches)
        buildPosesAndPaths();

        // ---------------- COMMAND SEQUENCE ----------------

        schedule(
                // Keep Pedro + ShotController updating every loop
                new RunCommand(() -> {
                    follower.update();
                    robot.shotController.updateState();
                }),

                new SequentialCommandGroup(
                        // 1) Start prepping the shot immediately so RPM/hood
                        //    can spin up while we rotate.
                        new InstantCommand(() -> {
                            robot.shotController.autoTargetShot();
                            robot.shotController.setState(ShotCycleState.PREPARE_SHOT);
                        }),

                        // 2) Use Pedro to rotate on the triangle to face the left goal
                        new FollowPathCommand(follower, rotateToGoal),

                        // 3) Micro line-up phase: fine-tune heading with LL/odom for 500ms
                        new RunCommand(() -> robot.autoAimStep())
                                .withTimeout(500),   // tune by feel


                        // 4) Now that we're lined up and PREPARE_SHOT has been running,
                        //    go to SHOOTING
                        new InstantCommand(() -> robot.shotController.setState(ShotCycleState.SHOOTING)),

                        // 5) Let 3 balls cycle through
                        new WaitCommand(2500),

                        // 6) Return to IDLE
                        new InstantCommand(() -> robot.shotController.setState(ShotCycleState.IDLE)),

                        // 7) Drive off the triangle
                        new FollowPathCommand(follower, driveOffTriangle, false)
                )

        );
    }

    // ---------------- LOOP ----------------

    @Override
    public void run() {
        super.run();

        telemetryData.addData("X (in)", follower.getPose().getX());
        telemetryData.addData("Y (in)", follower.getPose().getY());
        telemetryData.addData("Heading (rad)", follower.getPose().getHeading());
        telemetryData.update();
    }
}
