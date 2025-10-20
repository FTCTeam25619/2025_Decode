package org.firstinspires.ftc.teamcode.constants;

public class OpModes {
    public enum OpModeType {
        TELEOP,
        AUTO;
    }

    public enum OpModeSelection {
        PEDRO_SAMPLE_TELEOP(OpModeType.TELEOP),
        PEDRO_SAMPLE_AUTO(OpModeType.TELEOP),
        EXPLORATION(OpModeType.TELEOP);

        private OpModeType opModeType;

        private OpModeSelection(OpModeType opmodeTypeSetting){
            opModeType = opmodeTypeSetting;
        }

        public OpModeType getOpModeType(){
            return opModeType;
        }
    }
}
