package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.*;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "Red Near Auto 15 Balls")
public class RedNearAuto15Balls extends NearAutoBase {
    @Override
    protected double getGateGetBallPower() {
        return 0.6;
    }

    @Override
    protected NearPoses getNearPoses() {
        return new NearPoses(
                new Pose(startX, startY, startHeading),
                new Pose(shootX, shootY, shootHeading),
                new Pose(ball1StartX, ball1StartY, ball1StartHeading),
                new Pose(ball1EndX, ball1EndY, ball1EndHeading),
                new Pose(ball2StartX, ball2StartY, ball2StartHeading),
                new Pose(ball2EndX, ball2EndY, ball2EndHeading),
                new Pose(ball3StartX, ball3StartY, ball3StartHeading),
                new Pose(ball3EndX, ball3EndY, ball3EndHeading),
                new Pose(beforeGateX, beforeGateY, beforeGateHeading),
                new Pose(openGateX, openGateY, openGateHeading),
                new Pose(leaveX, leaveY, leaveHeading),
                new Pose(avoidX, avoidY, avoidHeading)
        );
    }
}
