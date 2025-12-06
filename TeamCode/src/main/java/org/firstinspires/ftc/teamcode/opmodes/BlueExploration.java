package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.OpModes;

@TeleOp(name = "Blue Exploration")
public class BlueExploration extends Exploration {
    public BlueExploration() {
        super(OpModes.Alliance.BLUE);
    }
}
