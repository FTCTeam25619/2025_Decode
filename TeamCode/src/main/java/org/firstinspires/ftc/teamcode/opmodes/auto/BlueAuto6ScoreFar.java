package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.autos.BlueFarFromGoalConstants;
import org.firstinspires.ftc.teamcode.autos.TimingConstants;
import org.firstinspires.ftc.teamcode.constants.Alliance;
import org.firstinspires.ftc.teamcode.lib.panels.PanelsUtils;

@Autonomous(name = "Blue 6-Score Far Wall", group = "Autonomous")
@Configurable // Panels
public class BlueAuto6ScoreFar extends PathFollowingAuto {
    private final BlueFarFromGoalConstants paths;

    public BlueAuto6ScoreFar() {
        super("Blue 6-Score Far Wall", Alliance.BLUE);
        this.paths = new BlueFarFromGoalConstants(follower);
    }

    public Pose getStartingPose() {
        return paths.blueFarWallStart.withHeading(Math.toRadians(paths.blueFarWallStartHeading));
    }

    public SequentialCommandGroup autoSequence() {
        return new SequentialCommandGroup(
            // Score preload
            setupFarWallPresetShot(),
            prepareShot(),
            new FollowPathCommand(follower, paths.blueWallToFarZonePrescore),
            new WaitCommand(TimingConstants.waitBeforeShootingMS),
            shoot(),
            new WaitCommand(TimingConstants.waitForShootingMS),
            // Intake 3 from row closest to far wall from goal
            intake(),
            new FollowPathCommand(follower, paths.blueFarZonePrescoreToFarIntake),
            new WaitCommand(TimingConstants.waitForIntakeMS),
            // Score 3
            setupFarWallPresetShot(),
            prepareShot(),
            new FollowPathCommand(follower, paths.blueFarWallIntakeToFarScore),
            new WaitCommand(TimingConstants.waitBeforeShootingMS),
            shoot(),
            new WaitCommand(TimingConstants.waitForShootingMS),
            // Park
            new FollowPathCommand(follower, paths.blueFarWallToPark, false),
            spinDown()
        );
    }

    public void drawPaths() {
        PanelsUtils.Drawing.drawPath(paths.blueWallToFarZonePrescore);
        PanelsUtils.Drawing.drawPath(paths.blueFarZonePrescoreToFarIntake);
        PanelsUtils.Drawing.drawPath(paths.blueFarWallIntakeToFarScore);
        PanelsUtils.Drawing.drawPath(paths.blueFarWallToPark);
    }
}
