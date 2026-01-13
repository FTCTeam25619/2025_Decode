package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.Alliance;

@TeleOp(name = "Red Exploration")
public class RedExploration extends Exploration {
    public RedExploration() {
        super(Alliance.RED);
    }
}
