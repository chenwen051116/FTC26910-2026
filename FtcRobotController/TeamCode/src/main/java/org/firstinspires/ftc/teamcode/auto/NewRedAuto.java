package org.firstinspires.ftc.teamcode.auto;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.*;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "New Red Auto 12 Balls")
public class NewRedAuto extends AutoBase{
    // START POINT
    private final Pose startPose = new Pose(startX, startY, startHeading);

    // SHOOT POINT
    private final Pose shootPose = new Pose(shootX, shootY, shootHeading);

    // BALL 1 START POINT
    private final Pose ball1StartPose = new Pose(ball1StartX, ball1StartY, ball1StartHeading);

    // BALL 1 END POINT
    private final Pose ball1EndPose = new Pose(ball1EndX, ball1EndY, ball1EndHeading);

    // BALL 2 START POINT
    private final Pose ball2StartPose = new Pose(ball2StartX, ball2StartY, ball2StartHeading);

    // BALL 2 END POINT
    private final Pose ball2EndPose = new Pose(ball2EndX, ball2EndY, ball2EndHeading);

    // BALL 2 AFTER POINT
    private final Pose ball2AfterPose = new Pose(ball2AfterX, ball2AfterY, ball2AfterHeading);
    // BALL 3 START POINT
    private final Pose ball3StartPose = new Pose(ball3StartX, ball3StartY, ball3StartHeading);

    // BALL 3 END POINT
    private final Pose ball3EndPose = new Pose(ball3EndX, ball3EndY, ball3EndHeading);

    // BEFORE GATE POSE
    private final Pose beforeGatePose = new Pose(beforeGateX, beforeGateY, beforeGateHeading);

    // OPEN GATE POSE
    private final Pose openGatePose = new Pose(openGateX, openGateY, openGateHeading);

    @Override
    public void initializePath() {
        // Initialize Sequencer
        // From Start pose to shooting pose
        shoot(startPose);

        // From shooting pose to get first ball
        intakeAtPos(shootPose, ball1StartPose, ball1EndPose);

        // Move from first ball pose to shooting
        shoot(ball1EndPose);

        // From shooting pose to get the second ball & open gate
        intakeAtPos(shootPose, ball2StartPose, ball2EndPose);

        // Move from second ball pose to shooting
        shoot(ball2EndPose);

        // Move from gate to third ball pose
        intakeAtPos(shootPose, ball3StartPose, ball3EndPose);

        // Move from third ball pose to shooting
        shoot(ball3EndPose);
    }
}
