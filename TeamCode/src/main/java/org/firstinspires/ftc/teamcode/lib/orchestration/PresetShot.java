package org.firstinspires.ftc.teamcode.lib.orchestration;

import org.firstinspires.ftc.teamcode.constants.HoodConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterFeedConstants;

public enum PresetShot {
    AtGoal(ShooterConstants.Targets.atGoalShot,
            HoodConstants.Targets.atGoalShot,
            ShooterFeedConstants.Targets.atGoalShot),
    NearZoneMid(ShooterConstants.Targets.nearZoneMidShot,
            HoodConstants.Targets.nearZoneMidShot,
            ShooterFeedConstants.Targets.nearZoneMidShot),
    NearZoneEnd(ShooterConstants.Targets.nearZoneEndShot,
            HoodConstants.Targets.nearZoneEndShot,
            ShooterFeedConstants.Targets.nearZoneEndShot),
    FarWall(ShooterConstants.Targets.farWallShot,
            HoodConstants.Targets.farWallShot,
            ShooterFeedConstants.Targets.farWallShot);

    public final double targetFlywheelRPM;
    public final double targetHoodAngle;
    public final double targetFeedPower;

    private PresetShot(double targetRPM, double targetAngle, double targetFeedPower) {
        this.targetFlywheelRPM = targetRPM;
        this.targetHoodAngle = targetAngle;
        this.targetFeedPower = targetFeedPower;
    }
}
