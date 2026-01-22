package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.autos.RedNearGoalConstants;
import org.firstinspires.ftc.teamcode.autos.TimingConstants;
import org.firstinspires.ftc.teamcode.constants.Alliance;
import org.firstinspires.ftc.teamcode.lib.panels.PanelsUtils;

@Autonomous(name = "Red 9-Score Near Goal", group = "Autonomous")
@Configurable // Panels
public class RedAuto9ScoreNear extends PathFollowingAuto {
    private final RedNearGoalConstants paths;

    // Mechanism commands - replace these with your actual subsystem commands
    public RedAuto9ScoreNear() {
        super("Red 9-Score Far Wall", Alliance.RED);
        this.paths = new RedNearGoalConstants(follower);
    }

    public Pose getStartingPose() {
        return paths.redNearWallStart.withHeading(Math.toRadians(paths.redNearWallStartHeading));
    }

    @Override
    public SequentialCommandGroup autoSequence() {
        return new SequentialCommandGroup( // Score preload
                setupNearWallPresetShot(),
                prepareShot(),
                new FollowPathCommand(follower, paths.redWallToNearZonePrescore),
                new WaitCommand(TimingConstants.waitBeforeShootingMS),
                shoot(),
                new WaitCommand(TimingConstants.waitForShootingMS),
                // Intake 3 from row closest to far wall from goal
                intake(),
                new FollowPathCommand(follower, paths.redNearZonePrescoreToNearIntake),
                new WaitCommand(TimingConstants.waitForIntakeMS),
                // Score 3
                setupNearWallPresetShot(),
                prepareShot(),
                new FollowPathCommand(follower, paths.redNearWallIntakeToNearScore),
                new WaitCommand(TimingConstants.waitBeforeShootingMS),
                shoot(),
                new WaitCommand(TimingConstants.waitForShootingMS),
                // Intake 3 from from mid row
                intake(),
                new FollowPathCommand(follower, paths.redNearZoneScoreToMidIntake),
                new WaitCommand(TimingConstants.waitForIntakeMS),
                // Score 3
                setupNearWallPresetShot(),
                prepareShot(),
                new FollowPathCommand(follower, paths.redMidWallIntakeToNearScore),
                new WaitCommand(TimingConstants.waitBeforeShootingMS),
                shoot(),
                new WaitCommand(TimingConstants.waitForShootingMS),
                // Park
                new FollowPathCommand(follower, paths.redNearWallToPark, false),
                spinDown()
        );
    }


    public void drawPaths() {
        PanelsUtils.Drawing.drawPath(paths.redWallToNearZonePrescore);
        PanelsUtils.Drawing.drawPath(paths.redNearZonePrescoreToNearIntake);
        PanelsUtils.Drawing.drawPath(paths.redNearWallIntakeToNearScore);
        PanelsUtils.Drawing.drawPath(paths.redNearWallToPark);
    }
}
