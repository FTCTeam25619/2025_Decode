package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.autos.RedFarFromGoalConstants;
import org.firstinspires.ftc.teamcode.autos.TimingConstants;
import org.firstinspires.ftc.teamcode.constants.Alliance;
import org.firstinspires.ftc.teamcode.lib.panels.PanelsUtils;

@Autonomous(name = "Red 9-Score Far Wall", group = "Autonomous")
@Configurable // Panels
public class RedAuto9ScoreFar extends PathFollowingAuto {
    private final RedFarFromGoalConstants paths;

    public RedAuto9ScoreFar() {
        super("Red 9-Score Far Wall", Alliance.RED);
        this.paths = new RedFarFromGoalConstants(follower);
    }

    public Pose getStartingPose() {
        return paths.redFarWallStart.withHeading(Math.toRadians(paths.redFarWallStartHeading));
    }

    @Override
    public SequentialCommandGroup autoSequence() {
        return new SequentialCommandGroup(  // Score preload
                setupFarWallPresetShot(),
                prepareShot(),
                new FollowPathCommand(follower, paths.redWallToFarZonePrescore),
                new WaitCommand(TimingConstants.waitBeforeShootingMS),
                shoot(),
                new WaitCommand(TimingConstants.waitForShootingMS),
                // Intake 3 from row closest to far wall from goal
                intake(),
                new FollowPathCommand(follower, paths.redFarZonePrescoreToFarIntake),
                new WaitCommand(TimingConstants.waitForIntakeMS),
                // Score 3
                setupFarWallPresetShot(),
                prepareShot(),
                new FollowPathCommand(follower, paths.redFarWallIntakeToFarScore),
                new WaitCommand(TimingConstants.waitBeforeShootingMS),
                shoot(),
                new WaitCommand(TimingConstants.waitForShootingMS),
                // Intake 3 from from mid row
                intake(),
                new FollowPathCommand(follower, paths.redFarZoneScoreToMidIntake),
                new WaitCommand(TimingConstants.waitForIntakeMS),
                // Score 3
                setupFarWallPresetShot(),
                prepareShot(),
                new FollowPathCommand(follower, paths.redMidWallIntakeToFarScore),
                new WaitCommand(TimingConstants.waitBeforeShootingMS),
                shoot(),
                new WaitCommand(TimingConstants.waitForShootingMS),
                // Park
                new FollowPathCommand(follower, paths.redFarWallToPark, false),
                spinDown()
        );
    }

    public void drawPaths() {
        PanelsUtils.Drawing.drawPath(paths.redWallToFarZonePrescore);
        PanelsUtils.Drawing.drawPath(paths.redFarZonePrescoreToFarIntake);
        PanelsUtils.Drawing.drawPath(paths.redFarWallIntakeToFarScore);
        PanelsUtils.Drawing.drawPath(paths.redFarWallToPark);
    }
}