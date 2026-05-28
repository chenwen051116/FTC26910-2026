package org.firstinspires.ftc.teamcode.auto;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.*;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "Red Auto 12 Balls")
public class RedAuto12Balls extends AutoBase {
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
        Pose ball2WithGatePose = new Pose(ball2WithGateX, ball2WithGateY, ball2WithGateHeading);

        // BALL 3 START POINT
        Pose ball3StartPose = new Pose(ball3StartX, ball3StartY, ball3StartHeading);

        // BALL 3 END POINT
        Pose ball3EndPose = new Pose(ball3EndX, ball3EndY, ball3EndHeading);


        // FROM START TO SHOOTING
        PathChain startToShootingPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .setGlobalDeceleration()
                .build();
        startToShootingPath.setConstraintsForAll(new PathConstraints(0.997, 100));

        // FROM SHOOTING TO BALL 1 BEGIN
        PathChain shootingToBall1StartPath = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, ball1StartPose))
                .setConstantHeadingInterpolation(ball1StartPose.getHeading())
                .setGlobalDeceleration()
                .build();
        shootingToBall1StartPath.setConstraintsForAll(new PathConstraints(0.997, 100));

        // FROM BALL 1 BEGIN TO BALL 1 END
        PathChain ball1StartToBall1EndPath = follower.pathBuilder()
                .addPath(new BezierLine(ball1StartPose, ball1EndPose))
                .setConstantHeadingInterpolation(ball1EndPose.getHeading())
                .setGlobalDeceleration()
                .build();
        ball1StartToBall1EndPath.setConstraintsForAll(new PathConstraints(0.997, 100));

        // FROM BALL 1 END TO SHOOTING
        PathChain ball1EndToShootingPath = follower.pathBuilder()
                .addPath(new BezierLine(ball1EndPose, shootPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .setGlobalDeceleration()
                .build();
        ball1EndToShootingPath.setConstraintsForAll(new PathConstraints(0.997, 100));

        // FROM SHOOTING TO BALL 2 BEGIN
        PathChain shootingToBall2StartPath = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, ball2StartPose))
                .setConstantHeadingInterpolation(ball2StartPose.getHeading())
                .setGlobalDeceleration()
                .build();
        shootingToBall2StartPath.setConstraintsForAll(new PathConstraints(0.997, 100));

        // FROM BALL 2 BEGIN TO BALL 2 END
        PathChain ball2StartToBall2WithGatePath = follower.pathBuilder()
                .addPath(new BezierLine(ball2StartPose, ball2WithGatePose))
                .setConstantHeadingInterpolation(ball2WithGatePose.getHeading())
                .setGlobalDeceleration()
                .build();
        ball2StartToBall2WithGatePath.setConstraintsForAll(new PathConstraints(0.997, 100));

        // FROM BALL 2 END TO SHOOTING
        PathChain ball2EndToShootingPath = follower.pathBuilder()
                .addPath(new BezierLine(ball2WithGatePose, shootPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .setGlobalDeceleration()
                .build();
        ball2EndToShootingPath.setConstraintsForAll(new PathConstraints(0.997, 100));

        // FROM SHOOTING TO BALL 3 BEGIN
        PathChain shootingToBall3StartPath = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, ball3StartPose))
                .setConstantHeadingInterpolation(ball3StartPose.getHeading())
                .setGlobalDeceleration()
                .build();
        shootingToBall3StartPath.setConstraintsForAll(new PathConstraints(0.997, 100));

        // FROM BALL 3 BEGIN TO BALL 3 END
        PathChain ball3StartToBall3EndPath = follower.pathBuilder()
                .addPath(new BezierLine(ball3StartPose, ball3EndPose))
                .setConstantHeadingInterpolation(ball3EndPose.getHeading())
                .setGlobalDeceleration()
                .build();
        ball3StartToBall3EndPath.setConstraintsForAll(new PathConstraints(0.997, 100));

        // FROM BALL 3 END TO SHOOTING
        PathChain ball3EndToShootingPath = follower.pathBuilder()
                .addPath(new BezierLine(ball3EndPose, shootPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .setGlobalDeceleration()
                .build();
        ball3EndToShootingPath.setConstraintsForAll(new PathConstraints(0.997, 100));

        // Initialize Sequencer
        // From Start pose to shooting pose
        shoot(startToShootingPath);

        // From shooting pose to get first ball
        intakeAtPos(shootingToBall1StartPath, ball1StartToBall1EndPath);

        // Move from first ball pose to shooting
        shoot(ball1EndToShootingPath);

        // From shooting pose to get the second ball & open gate
        intakeAtPos(shootingToBall2StartPath, ball2StartToBall2WithGatePath);

        // Move from second ball pose to shooting
        shoot(ball2EndToShootingPath);

        // Move from gate to third ball pose
        intakeAtPos(shootingToBall3StartPath, ball3StartToBall3EndPath);

        // Move from third ball pose to shooting
        shoot(ball3EndToShootingPath);
    }
}
