package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedFar.*;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "Red Far Auto Default Path")
public class RedFarAutoDefaultPath extends FarAutoDefaultPathBase {
    @Override
    protected FarPoses getFarPoses() {
        return new FarPoses(
                new Pose(startX, startY, startHeading),
                new Pose(shootX, shootY, shootHeading),
                new Pose(ball3StartX, ball3StartY, ball3StartHeading),
                new Pose(ball3EndX, ball3EndY, ball3EndHeading),
                new Pose(humanZoneGetStartX, humanZoneGetStartY, humanZoneGetStartHeading),
                new Pose(humanZoneGetEndX, humanZoneGetEndY, humanZoneGetEndHeading),
                new Pose(leaveX, leaveY, leaveHeading)
        );
    }
}
