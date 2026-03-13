package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;

public class NewRedAuto extends AutoBase{
    public static double defaultMoveMaxPower = 0.8, intakeDefaultMoveMaxPower = 0.6;
    public static double intakeBreakingStrength = 1, shootingBreakingStrength = 0.8;

    // START POINT
    public static double startX = 117.8501, startY = 107.7211, startHeading = 0.76;
    private final Pose startPose = new Pose(startX, startY, startHeading);

    // SHOOT POINT
    public static double shootX = 91, shootY = 91, shootHeading = 0.76;
    private final Pose shootPose = new Pose(shootX, shootY, shootHeading);

    // BALL 1 START POINT
    public static double ball1StartX = 84, ball1StartY = 70, ball1StartHeading = 0;
    private final Pose ball1StartPose = new Pose(ball1StartX, ball1StartY, ball1StartHeading);

    // BALL 1 END POINT
    public static double ball1EndX = 115, ball1EndY = 70, ball1EndHeading = 0;
    private final Pose ball1EndPose = new Pose(ball1EndX, ball1EndY, ball1EndHeading);

    // BALL 2 START POINT
    public static double ball2StartX = 84, ball2StartY = 45, ball2StartHeading = 0;
    private final Pose ball2StartPose = new Pose(ball2StartX, ball2StartY, ball2StartHeading);

    // BALL 2 END POINT
    public static double ball2EndX = 120, ball2EndY = 45, ball2EndHeading = 0;
    private final Pose ball2EndPose = new Pose(ball2EndX, ball2EndY, ball2EndHeading);

    // BALL 2 AFTER POINT
    public static double ball2AfterX = 108, ball2AfterY = 45, ball2AfterHeading = 0;
    private final Pose ball2AfterPose = new Pose(ball2AfterX, ball2AfterY, ball2AfterHeading);


    // BALL 3 START POINT
    public static double ball3StartX = 84, ball3StartY = 20, ball3StartHeading = 0;
    private final Pose ball3StartPose = new Pose(ball3StartX, ball3StartY, ball3StartHeading);

    // BALL 3 END POINT
    public static double ball3EndX = 115, ball3EndY = 20, ball3EndHeading = 0;
    private final Pose ball3EndPose = new Pose(ball3EndX, ball3EndY, ball3EndHeading);

    // BEFORE GATE POSE
    public static double beforeGateX = 96.5, beforeGateY = 67.5, beforeGateHeading = 0;
    private final Pose beforeGatePose = new Pose(beforeGateX, beforeGateY, beforeGateHeading);

    // OPEN GATE POSE
    public static double openGateX = 126.86, openGateY = 46.23, openGateHeading = 0.1157;
    private final Pose openGatePose = new Pose(openGateX, openGateY, openGateHeading);

    @Override
    public void initializePath() {
        // Initialize Sequencer
        // From Start pose to shooting pose
        shoot();

        // From shooting pose to get first ball
        intakeAtPos(ball1StartPose, ball1EndPose);

        // Move from first ball pose to shooting
        shoot();

        // From shooting pose to get the second ball & open gate
        intakeAtPos(ball2StartPose, ball2EndPose);

        // Move away from ball 2
        sequencer.run(() -> drivetrain.goTo(ball2AfterPose));
        sequencer.waitUntil(() -> !drivetrain.followerIsBusy());

        // Move from second ball pose to shooting
        shoot();

        // Move from gate to third ball pose
        intakeAtPos(ball3StartPose, ball3EndPose);

        // Move from third ball pose to shooting
        shoot();
    }
}
