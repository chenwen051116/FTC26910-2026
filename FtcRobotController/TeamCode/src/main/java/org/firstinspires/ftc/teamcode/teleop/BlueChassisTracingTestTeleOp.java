package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Blue Chassis Tracing Teleop")
public class BlueChassisTracingTestTeleOp extends AllianceTeleOp {
    @Override
    protected boolean isRedAlliance() {
        return false;
    }

    @Override
    protected Pose initialPose() {
        return new Pose(9, 9, 0);
    }
}
