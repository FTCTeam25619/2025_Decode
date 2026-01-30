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

@Autonomous(name = "Blue 6-Score Near Goal", group = "Autonomous")
@Configurable // Panels
public class BlueAuto6ScoreNear extends PathFollowingAuto {
    private final BlueNearGoalConstants paths;

    public BlueAuto6ScoreNear() {
        super("Blue 6-Score Near Goal", Alliance.BLUE);
        this.paths = new BlueNearGoalConstants(follower);
    }

    public Pose getStartingPose() {
        return paths.blueNearWallStart.withHeading(Math.toRadians(paths.blueNearWallStartHeading));
    }

    public SequentialCommandGroup autoSequence() {

        return new SequentialCommandGroup(
                // Score preload
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
                // Park
                new FollowPathCommand(follower, paths.blueNearWallToPark, false),
                spinDown()
        );
    }
//        readyToShootPreload().whenActive(shoot());

    public void drawPaths() {
        PanelsUtils.Drawing.drawPath(paths.blueWallToNearZonePrescore);
        PanelsUtils.Drawing.drawPath(paths.blueNearZonePrescoreToNearIntake);
        PanelsUtils.Drawing.drawPath(paths.blueNearWallIntakeToNearScore);
        PanelsUtils.Drawing.drawPath(paths.blueNearWallToPark);
    }
}