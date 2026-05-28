package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Blue TeleOp")
public class BlueTeleOp extends AllianceTeleOp {
    @Override
    protected boolean isRedAlliance() {
        return false;
    }
}