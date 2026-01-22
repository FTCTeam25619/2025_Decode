package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.autos.BlueNearGoalConstants;
import org.firstinspires.ftc.teamcode.autos.TimingConstants;
import org.firstinspires.ftc.teamcode.constants.Alliance;
import org.firstinspires.ftc.teamcode.lib.panels.PanelsUtils;

@Autonomous(name = "Blue 9-Score Near Goal", group = "Autonomous")
@Configurable // Panels
public class BlueAuto9ScoreNear extends PathFollowingAuto {
    private final BlueNearGoalConstants paths;

    // Mechanism commands - replace these with your actual subsystem commands
    public BlueAuto9ScoreNear() {
        super("Blue 9-Score Near Goal", Alliance.BLUE);
        this.paths = new BlueNearGoalConstants(follower);
    }

    public Pose getStartingPose() {
        return paths.blueNearWallStart.withHeading(Math.toRadians(paths.blueNearWallStartHeading));
    }

    @Override
    public SequentialCommandGroup autoSequence() {
        return new SequentialCommandGroup( // Score preload
                setupNearWallPresetShot(),
                prepareShot(),
                new FollowPathCommand(follower, paths.blueWallToNearZonePrescore),
                new WaitCommand(TimingConstants.waitBeforeShootingMS),
                shoot(),
                new WaitCommand(TimingConstants.waitForShootingMS),
                // Intake 3 from row closest to far wall from goal
                intake(),
                new FollowPathCommand(follower, paths.blueNearZonePrescoreToNearIntake),
                new WaitCommand(TimingConstants.waitForIntakeMS),
                // Score 3
                setupNearWallPresetShot(),
                prepareShot(),
                new FollowPathCommand(follower, paths.blueNearWallIntakeToNearScore),
                new WaitCommand(TimingConstants.waitBeforeShootingMS),
                shoot(),
                new WaitCommand(TimingConstants.waitForShootingMS),
                // Intake 3 from from mid row
                intake(),
                new FollowPathCommand(follower, paths.blueNearZoneScoreToMidIntake),
                new WaitCommand(TimingConstants.waitForIntakeMS),
                // Score 3
                setupNearWallPresetShot(),
                prepareShot(),
                new FollowPathCommand(follower, paths.blueMidWallIntakeToNearScore),
                new WaitCommand(TimingConstants.waitBeforeShootingMS),
                shoot(),
                new WaitCommand(TimingConstants.waitForShootingMS),
                // Park
                new FollowPathCommand(follower, paths.blueNearWallToPark, false),
                spinDown()
        );
    }


    public void drawPaths() {
        PanelsUtils.Drawing.drawPath(paths.blueWallToNearZonePrescore);
        PanelsUtils.Drawing.drawPath(paths.blueNearZonePrescoreToNearIntake);
        PanelsUtils.Drawing.drawPath(paths.blueNearWallIntakeToNearScore);
        PanelsUtils.Drawing.drawPath(paths.blueNearZoneScoreToMidIntake);
        PanelsUtils.Drawing.drawPath(paths.blueMidWallIntakeToNearScore);
        PanelsUtils.Drawing.drawPath(paths.blueNearWallToPark);
    }
}
