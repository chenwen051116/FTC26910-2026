//package org.firstinspires.ftc.teamcode.auto;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.*;
//
//
//import com.acmerobotics.dashboard.config.Config;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//@Config
//@Autonomous(name = "New Red Auto 15 Balls")
//public class NewRedAuto15Balls extends AutoBase{
//    // START POINT
//    private final Pose startPose = new Pose(startX, startY, startHeading);
//
//    // SHOOT POINT
//    private final Pose shootPose = new Pose(shootX, shootY, shootHeading);
//
//
//    // BALL 1 START POINT
//    private final Pose ball1StartPose = new Pose(ball1StartX, ball1StartY, ball1StartHeading);
//
//    // BALL 1 END POINT
//    private final Pose ball1EndPose = new Pose(ball1EndX, ball1EndY, ball1EndHeading);
//
//    // BALL 2 START POINT
//    private final Pose ball2StartPose = new Pose(ball2StartX, ball2StartY, ball2StartHeading);
//
//    // BALL 2 END POINT
//    private final Pose ball2EndPose = new Pose(ball2EndX, ball2EndY, ball2EndHeading);
//
//    // BALL 2 AFTER POINT
//    private final Pose ball2AfterPose = new Pose(ball2AfterX, ball2AfterY, ball2AfterHeading);
//
//
//    // BALL 3 START POINT
//    private final Pose ball3StartPose = new Pose(ball3StartX, ball3StartY, ball3StartHeading);
//
//    // BALL 3 END POINT
//    private final Pose ball3EndPose = new Pose(ball3EndX, ball3EndY, ball3EndHeading);
//
//    // BEFORE GATE POSE
//    private final Pose beforeGatePose = new Pose(beforeGateX, beforeGateY, beforeGateHeading);
//
//    // OPEN GATE POSE
//    private final Pose openGatePose = new Pose(openGateX, openGateY, openGateHeading);
//
//    private final Pose getGateBallPose = new Pose(getGateBallX, getGateBallY, getGateBallHeading);
//
//    @Override
//    public void initializePath() {
//        // Initialize Sequencer
//        // From Start pose to shooting pose
//        shoot(startPose);
//
//        // From shooting pose to get the second ball
//        intakeAtPos(shootPose, ball2StartPose, ball2EndPose);
//
//        // Move away from ball 2
//        sequencer.run(() -> drivetrain.followPath(ball2EndPose, ball2AfterPose));
//        sequencer.waitUntil(() -> !drivetrain.followerIsBusy());
//
//        // Move from second ball pose to shooting
//        shoot(ball2AfterPose);
//
//
//        // Open gate
//        sequencer.run(() -> drivetrain.followPath(shootPose, beforeGatePose));
//        sequencer.waitUntil(() -> !drivetrain.followerIsBusy());
//
//        intakeAtPos(beforeGatePose, openGatePose, getGateBallPose);
//
//        // From gate pose to shooting pose
//        shoot(getGateBallPose);
//
//        // Move from gate to first ball pose
//        intakeAtPos(shootPose, ball1StartPose, ball1EndPose);
//
//        // Move from first ball pose to shooting
//        shoot(ball1EndPose);
//        // Move from gate to first ball pose
//        intakeAtPos(shootPose, ball3StartPose, ball3EndPose);
//
//        // Move from first ball pose to shooting
//        shoot(ball3EndPose);
//    }
//}
