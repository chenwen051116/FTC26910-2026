package org.firstinspires.ftc.teamcode.auto;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.BlueNear.*;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "Blue Near Auto 15 Balls")
public class BlueNearAuto15Balls extends AutoBase{
    public static double startToShootingBrakingStart = 1.4;
    public static double ball2IntakeBrakingStart = 1.7;
    public static double ball3IntakeBrakingStart = 1.8;
    public static double ball1EndToShootingBrakingStart = 1.7;
    public static double avoidToShootingBrakingStart = 1.75;
    public static double ball3EndToShootingBrakingStart = 1.8;
    public static double gateTValue = 0.987;
    public static double shootingTValue = 0.99;
    public static double avoidTValue = 0.987;
    public static double gateGetBallPower = 0.8;
    public static double ball2IntakeMaxPower = 0.8;
    public static double ball2IntakeBrakingStrength = 0.6;
    public static double ball3EndToShootingBrakingStrength = 0.6;
    public static double avoidToShootingBrakingStrength = 0.7;
    public static double avoidToShootingMaxPower = 0.8;

    @Override
    public void setStartPose() {
        this.startPose = new Pose(startX, startY, startHeading);
    }

    @Override
    public void setShootPose() {
        this.shootPose = new Pose(shootX, shootY, shootHeading);
    }

    @Override
    public void setIsRed() {
        this.isRed = false;
    }

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

        // AVOID POINT
        Pose avoidPose = new Pose(avoidX, avoidY, avoidHeading);

        // BALL 3 START POINT
        Pose ball3StartPose = new Pose(ball3StartX, ball3StartY, ball3StartHeading);

        // BALL 3 END POINT
        Pose ball3EndPose = new Pose(ball3EndX, ball3EndY, ball3EndHeading);

        // BEFORE GATE POSE
        Pose beforeGatePose = new Pose(beforeGateX, beforeGateY, beforeGateHeading);

        // OPEN GATE POSE
        Pose openGatePose = new Pose(openGateX, openGateY, openGateHeading);

        // LEAVE POSE
        Pose leavePose = new Pose(leaveX, leaveY, leaveHeading);

        // FROM START TO SHOOTING
        PathChain startToShootingPath = buildPathLinearInterpol(startPose,
                shootPose,
                defaultBrakingStrength,
                startToShootingBrakingStart,
                shootingTValue
        );

        // FROM SHOOTING TO BALL 2 BEGIN
        PathChain shootingToBall2StartPath = buildPath(shootPose,
                ball2StartPose,
                ball2IntakeBrakingStrength,
                ball2IntakeBrakingStart
        );

        // FROM BALL 2 BEGIN TO BALL 2 END
        PathChain ball2StartToBall2EndPath = buildPathLinearInterpol(ball2StartPose, ball2EndPose);

        // FROM BALL 2 END TO AVOID POSE
        PathChain ball2EndToAvoidPath = follower.pathBuilder()
                .addPath(new BezierLine(ball2EndPose, avoidPose))
                .setLinearHeadingInterpolation(ball2EndHeading, avoidHeading)
                .setTValueConstraint(avoidTValue)
                .build();

        // FROM AVOID TO SHOOT POSE
        PathChain avoidToShootPath = buildPathLinearInterpol(
                avoidPose,
                shootPose,
                avoidToShootingBrakingStrength,
                avoidToShootingBrakingStart
        );

        // FROM SHOOTING TO BEFORE GATE POSE
        PathChain shootingToBeforeGatePath = buildPath(shootPose, beforeGatePose);

        // FROM BEFORE GATE POSE TO OPEN GATE POSE
        PathChain beforeGateToOpenGatePath = buildPath(
                beforeGatePose,
                openGatePose,
                defaultBrakingStrength,
                defaultBrakingStart,
                gateTValue
        );

        // FROM OPEN GATE POSE TO SHOOTING POSE
        PathChain openGatePoseToAvoidPath = follower.pathBuilder()
                .addPath(new BezierLine(openGatePose, avoidPose))
                .setLinearHeadingInterpolation(ball2EndHeading, avoidHeading)
                .setTValueConstraint(avoidTValue)
                .build();

        // FROM SHOOTING TO BALL 1 BEGIN
        PathChain shootingToBall1StartPath = buildPath(shootPose, ball1StartPose);

        // FROM BALL 1 BEGIN TO BALL 1 END
        PathChain ball1StartToBall1EndPath = buildPathLinearInterpol(ball1StartPose, ball1EndPose);

        // FROM BALL 1 END TO SHOOTING
        PathChain ball1EndToShootingPath = buildPath(ball1EndPose,
                shootPose,
                defaultBrakingStrength,
                ball1EndToShootingBrakingStart);

        // FROM SHOOTING TO BALL 3 BEGIN
        PathChain shootingToBall3StartPath = buildPath(shootPose,
                ball3StartPose,
                defaultBrakingStrength,
                ball3IntakeBrakingStart
        );

        // FROM BALL 3 BEGIN TO BALL 3 END
        PathChain ball3StartToBall3EndPath = buildPathLinearInterpol(ball3StartPose, ball3EndPose);

        // FROM BALL 3 END TO SHOOTING
        PathChain ball3EndToShootingPath = buildPath(ball3EndPose,
                shootPose,
                ball3EndToShootingBrakingStrength,
                ball3EndToShootingBrakingStart);

        // FROM SHOOTING TO LEAVE
        PathChain shootingToLeavePath = buildPath(shootPose, leavePose);

        // Initialize Sequencer
        // From start pose to shooting pose
        shoot(startToShootingPath);

        // From shooting pose to ball 2 pose
        intakeAtPos(shootingToBall2StartPath, ball2StartToBall2EndPath, ball2IntakeMaxPower);

        // From ball 2 end to shooting
        goTo(ball2EndToAvoidPath, defaultMoveMaxPower, false);
        shoot(avoidToShootPath, avoidToShootingMaxPower);

        // From shooting to gate
        goTo(shootingToBeforeGatePath);

        // Intake
        intakeToPos(beforeGateToOpenGatePath, defaultIntakeDuration, gateGetBallPower);

        // From before gate pose to shooting pose
        goTo(openGatePoseToAvoidPath, defaultMoveMaxPower, false);
        shoot(avoidToShootPath);

        // From shooting pose to ball 1 pose
        intakeAtPos(shootingToBall1StartPath, ball1StartToBall1EndPath);

        // From ball 1 end to shooting pose
        shoot(ball1EndToShootingPath);

        // From shooting pose to ball 3
        intakeAtPos(shootingToBall3StartPath, ball3StartToBall3EndPath);

        // From ball 3 end to shooting pose
        shoot(ball3EndToShootingPath);

        // From shooting pose to leave pose
        goTo(shootingToLeavePath);
    }
}
