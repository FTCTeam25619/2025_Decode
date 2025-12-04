package org.firstinspires.ftc.teamcode.lib.orchestration;

import org.firstinspires.ftc.teamcode.constants.HoodConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;

public enum PresetShot {
    AtGoal(ShooterConstants.Targets.atGoalShot, HoodConstants.Targets.atGoalShot),
    NearZoneMid(ShooterConstants.Targets.nearZoneMidShot, HoodConstants.Targets.nearZoneMidShot),
    NearZoneEnd(ShooterConstants.Targets.nearZoneEndShot, HoodConstants.Targets.nearZoneEndShot),
    FarWall(ShooterConstants.Targets.farWallShot, HoodConstants.Targets.farWallShot);

    public final double targetFlywheelRPM;
    public final double targetHoodAngle;

    private PresetShot(double targetRPM, double targetAngle) {
        this.targetFlywheelRPM = targetRPM;
        this.targetHoodAngle = targetAngle;
    }
}
