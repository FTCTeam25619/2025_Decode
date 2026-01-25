package org.firstinspires.ftc.teamcode.constants;

public class IndicatorLightsConstants {
    public enum IndicatorColors {
        GREEN(0.500),
        PURPLE(0.713),
        RED(0.290),
        ORANGE(0.333),
        YELLOW(0.388),
        BLUE(0.590),
        WHITE(1.0),
        OFF(0.0);

        public final double pwmSetting;

        private IndicatorColors(double setting) {
            pwmSetting = setting;
        }
    }
}
