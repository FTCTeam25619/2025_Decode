package org.firstinspires.ftc.teamcode.lib;

import org.firstinspires.ftc.teamcode.constants.ConfigConstants;

public enum GamePieceColor {
    PURPLE(ConfigConstants.Colors.PURPLE_HUE_MIN, ConfigConstants.Colors.PURPLE_HUE_MAX),
    GREEN(ConfigConstants.Colors.GREEN_HUE_MIN, ConfigConstants.Colors.GREEN_HUE_MAX);

    public final float hueMin;
    public final float hueMax;

    private GamePieceColor(float hueMin, float hueMax) {
        this.hueMin = hueMin;
        this.hueMax = hueMax;
    }

    public boolean matches(float hue) {
        return hue >= hueMin && hue <= hueMax;
    }

    public static GamePieceColor colorOf(float hue) {
        for (GamePieceColor color: GamePieceColor.values()) {
            if (color.matches(hue)) {
                return color;
            }
        }
        return null;
    }
}
