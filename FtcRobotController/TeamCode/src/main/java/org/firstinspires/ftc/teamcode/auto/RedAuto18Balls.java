//package org.firstinspires.ftc.teamcode.auto;
//
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball1EndHeading;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball1EndX;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball1EndY;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball1StartHeading;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball1StartX;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball1StartY;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball2AfterHeading;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball2AfterX;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball2AfterY;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball2EndHeading;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball2EndX;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball2EndY;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball2StartHeading;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball2StartX;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball2StartY;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball3EndHeading;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball3EndX;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball3EndY;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball3StartHeading;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball3StartX;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball3StartY;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.beforeGateHeading;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.beforeGateX;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.beforeGateY;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.openGateHeading;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.openGateX;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.openGateY;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.shootHeading;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.shootX;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.shootY;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.startHeading;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.startX;
//import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.startY;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//@Config
//@Autonomous(name = "!!INCOMPLETE!! Red Auto 18 Balls")
//public class RedAuto18Balls extends AutoBase{
//
//    public static double ball2IntakeBrakingStart = 1.3;
//    public static double ball3IntakeBrakingStart = 1.6;
//    public static double gateTValue = 0.99;
//
//    @Override
//    public void initializePath() {
//        // START POINT
//        Pose startPose = new Pose(startX, startY, startHeading);
//
//        // SHOOT POINT
//        Pose shootPose = new Pose(shootX, shootY, shootHeading);
//
//
//        // BALL 1 START POINT
//        Pose ball1StartPose = new Pose(ball1StartX, ball1StartY, ball1StartHeading);
//
//        // BALL 1 END POINT
//        Pose ball1EndPose = new Pose(ball1EndX, ball1EndY, ball1EndHeading);
//
//        // BALL 2 START POINT
//        Pose ball2StartPose = new Pose(ball2StartX, ball2StartY, ball2StartHeading);
//
//        // BALL 2 END POINT
//        Pose ball2EndPose = new Pose(ball2EndX, ball2EndY, ball2EndHeading);
//
//        // BALL 2 AFTER POINT
//        Pose ball2AfterPose = new Pose(ball2AfterX, ball2AfterY, ball2AfterHeading);
//
//
//        // BALL 3 START POINT
//        Pose ball3StartPose = new Pose(ball3StartX, ball3StartY, ball3StartHeading);
//
//        // BALL 3 END POINT
//        Pose ball3EndPose = new Pose(ball3EndX, ball3EndY, ball3EndHeading);
//
//        // BEFORE GATE POSE
//        Pose beforeGatePose = new Pose(beforeGateX, beforeGateY, beforeGateHeading);
//
//        // OPEN GATE POSE
//        Pose openGatePose = new Pose(openGateX, openGateY, openGateHeading);
//
//
//        // FROM START TO SHOOTING
//        PathChain startToShootingPath = buildPath(startPose, shootPose, HeadingInterpolation.LINEAR);
//
//        // FROM SHOOTING TO BALL 2 BEGIN
//        PathChain shootingToBall2StartPath = buildPath(shootPose,
//                ball2StartPose,
//                defaultBrakingStrength,
//                ball2IntakeBrakingStart
//        );
//
//        // FROM BALL 2 BEGIN TO BALL 2 END
//        PathChain ball2StartToBall2EndPath = buildPath(ball2StartPose, ball2EndPose, HeadingInterpolation.LINEAR);
//
//        // FROM BALL 2 END TO BALL 2 AFTER
//        PathChain ball2EndToBall2AfterPath = buildPath(ball2EndPose, ball2AfterPose);
//
//        // FROM BALL 2 AFTER TO SHOOTING
//        PathChain ball2AfterToShootingPath = buildPath(ball2AfterPose, shootPose);
//
//        // FROM BALL 2 END TO SHOOTING
//        PathChain ball2EndToShootingPath = buildPath(ball2EndPose, shootPose);
//
//        // FROM SHOOTING TO BEFORE GATE POSE
//        PathChain shootingToBeforeGatePath = buildPath(shootPose, beforeGatePose);
//
//        // FROM BEFORE GATE POSE TO OPEN GATE POSE
//        PathChain beforeGateToOpenGatePath = buildPath(beforeGatePose, openGatePose);
//
//        // FROM OPEN GATE POSE TO GATE INTAKE POSE
//        PathChain openGateToGetGateBallPath = buildPath(openGatePose,
//                getGateBallPose,
//                defaultBrakingStrength,
//                defaultBrakingStart,
//                gateTValue
//                );
//
//        // FROM GATE INTAKE POSE TO BEFORE GATE POSE
//        PathChain getGateBallToBeforeGatePath = buildPath(getGateBallPose, beforeGatePose);
//
//        // FROM BEFORE GATE POSE TO SHOOTING POSE
//        PathChain beforeGateToShootingPath = buildPath(beforeGatePose, shootPose);
//
//        // FROM GATE INTAKE POS TO SHOOTING POSE
//        PathChain getGateBallToShootingPath = buildPath(getGateBallPose, shootPose);
//
//        // FROM SHOOTING TO BALL 1 BEGIN
//        PathChain shootingToBall1StartPath = buildPath(shootPose, ball1StartPose);
//
//        // FROM BALL 1 BEGIN TO BALL 1 END
//        PathChain ball1StartToBall1EndPath = buildPath(ball1StartPose, ball1EndPose, HeadingInterpolation.LINEAR);
//
//        // FROM BALL 1 END TO SHOOTING
//        PathChain ball1EndToShootingPath = buildPath(ball1EndPose, shootPose);
//
//        // FROM SHOOTING TO BALL 3 BEGIN
//        PathChain shootingToBall3StartPath = buildPath(shootPose,
//                ball3StartPose,
//                defaultBrakingStrength,
//                ball3IntakeBrakingStart
//        );
//
//        // FROM BALL 3 BEGIN TO BALL 3 END
//        PathChain ball3StartToBall3EndPath = buildPath(ball3StartPose, ball3EndPose, HeadingInterpolation.LINEAR);
//
//        // FROM BALL 3 END TO SHOOTING
//        PathChain ball3EndToShootingPath = buildPath(ball3EndPose, shootPose);
//
//        // Initialize Sequencer
//        // From start pose to shooting pose
//        shoot(startToShootingPath);
//
//        // From shooting pose to ball 2 pose
//        intakeAtPos(shootingToBall2StartPath, ball2StartToBall2EndPath);
//
//        // Move away from ball 2
//        //goTo(ball2EndToBall2AfterPath);
//
//        // From ball2 after to shooting
//        //shoot(ball2AfterToShootingPath);
//
//        // From ball 2 end to shooting
//        shoot(ball2EndToShootingPath);
//
//        // From shooting to gate
//        goTo(shootingToBeforeGatePath);
//
//        // Open Gate
//        goTo(beforeGateToOpenGatePath, 0.5);
//
//        // Intake from gate
//        intakeToPos(openGateToGetGateBallPath);
//
//        // From gate get ball pose to shooting pose
//        shoot(getGateBallToShootingPath);
//
//        // From shooting to gate
//        goTo(shootingToBeforeGatePath);
//
//        // Open Gate
//        goTo(beforeGateToOpenGatePath, 0.5);
//
//        // Intake from gate
//        intakeToPos(openGateToGetGateBallPath);
//
//        // From gate get ball pose to shooting pose
//        shoot(getGateBallToShootingPath);
//
//        // From shooting pose to ball 1 pose
//        intakeAtPos(shootingToBall1StartPath, ball1StartToBall1EndPath);
//
//        // From ball 1 end to shooting pose
//        shoot(ball1EndToShootingPath);
//
//        // From shooting pose to ball 3
//        intakeAtPos(shootingToBall3StartPath, ball3StartToBall3EndPath);
//
//        // From ball 3 end to shooting pose
//        shoot(ball3EndToShootingPath);
//    }
//}
