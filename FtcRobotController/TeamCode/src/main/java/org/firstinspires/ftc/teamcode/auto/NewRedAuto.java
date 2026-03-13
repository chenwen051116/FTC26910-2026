package org.firstinspires.ftc.teamcode.auto;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.*;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "New Red Auto 12 Balls")
public class NewRedAuto extends OldAutoBase {
    @Override
    public void initializePath() {
        // START POINT
        Pose startPose = new Pose(startX, startY, startHeading);

        // SHOOT POINT
        Pose shootPose = new Pose(shootX, shootY, shootHeading);

        // BALL 1 START POINT
        Pose ball1StartPose = new Pose(ball1StartX, ball1StartY, ball1StartHeading);

        // BALL 1 END POINT
        Pose ball1EndPose = new Pose(ball1EndX, ball1EndY, ball1EndHeading);

        // BALL 2 START POINT
        Pose ball2StartPose = new Pose(ball2StartX, ball2StartY, ball2StartHeading);

        // BALL 2 END POINT
        Pose ball2EndPose = new Pose(ball2EndX, ball2EndY, ball2EndHeading);

        // BALL 3 START POINT
        Pose ball3StartPose = new Pose(ball3StartX, ball3StartY, ball3StartHeading);

        // BALL 3 END POINT
        Pose ball3EndPose = new Pose(ball3EndX, ball3EndY, ball3EndHeading);


        // FROM START TO SHOOTING
        PathChain startToShootingPath = buildShootingPath(startPose, shootPose);

        // FROM SHOOTING TO BALL 1 BEGIN
        PathChain shootingToBall1StartPath = buildIntakePath(shootPose, ball1StartPose);

        // FROM BALL 1 BEGIN TO BALL 1 END
        PathChain ball1StartToBall1EndPath = buildIntakePath(ball1StartPose, ball1EndPose);

        // FROM BALL 1 END TO SHOOTING
        PathChain ball1EndToShootingPath = buildShootingPath(ball1EndPose, shootPose);

        // FROM SHOOTING TO BALL 2 BEGIN
        PathChain shootingToBall2StartPath = buildIntakePath(shootPose, ball2StartPose);

        // FROM BALL 2 BEGIN TO BALL 2 END
        PathChain ball2StartToBall2EndPath = buildIntakePath(ball2StartPose, ball2EndPose);

        // FROM BALL 2 END TO SHOOTING
        PathChain ball2EndToShootingPath = buildShootingPath(ball2EndPose, shootPose);

        // FROM SHOOTING TO BALL 3 BEGIN
        PathChain shootingToBall3StartPath = buildIntakePath(shootPose, ball3StartPose);

        // FROM BALL 3 BEGIN TO BALL 3 END
        PathChain ball3StartToBall3EndPath = buildIntakePath(ball3StartPose, ball3EndPose);

        // FROM BALL 3 END TO SHOOTING
        PathChain ball3EndToShootingPath = buildShootingPath(ball3EndPose, shootPose);
        // Initialize Sequencer
        // From Start pose to shooting pose
        shoot(startToShootingPath);

        // From shooting pose to get first ball
        intakeAtPos(shootingToBall1StartPath, ball1StartToBall1EndPath);

        // Move from first ball pose to shooting
        shoot(ball1EndToShootingPath);

        // From shooting pose to get the second ball & open gate
        intakeAtPos(shootingToBall2StartPath, ball2StartToBall2EndPath);

        // Move from second ball pose to shooting
        shoot(ball2EndToShootingPath);

        // Move from gate to third ball pose
        intakeAtPos(shootingToBall3StartPath, ball3StartToBall3EndPath);

        // Move from third ball pose to shooting
        shoot(ball3EndToShootingPath);
    }
}
