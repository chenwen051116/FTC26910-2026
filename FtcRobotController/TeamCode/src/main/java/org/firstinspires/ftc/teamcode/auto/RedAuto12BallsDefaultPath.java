package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.*;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "Red Auto 12 Balls Default Path")
public class RedAuto12BallsDefaultPath extends AutoBase {
    @Override
    public void followPath(PathChain pathChain) {
        follower.followPath(pathChain);
    }

    @Override
    public void followPath(PathChain pathChain, double maxPower) {
        follower.followPath(pathChain);
    }

    @Override
    public void followPath(PathChain pathChain, double maxPower, boolean holdEnd) {
        follower.followPath(pathChain);
    }

    @Override
    public void initializePath() {
        Pose startPose = new Pose(startX, startY, startHeading);
        Pose shootPose = new Pose(shootX, shootY, shootHeading);
        Pose ball1StartPose = new Pose(ball1StartX, ball1StartY, ball1StartHeading);
        Pose ball1EndPose = new Pose(ball1EndX, ball1EndY, ball1EndHeading);
        Pose ball2StartPose = new Pose(ball2StartX, ball2StartY, ball2StartHeading);
        Pose ball2WithGatePose = new Pose(ball2WithGateX, ball2WithGateY, ball2WithGateHeading);
        Pose ball3StartPose = new Pose(ball3StartX, ball3StartY, ball3StartHeading);
        Pose ball3EndPose = new Pose(ball3EndX, ball3EndY, ball3EndHeading);

        PathChain startToShootingPath = lineConstant(startPose, shootPose, shootPose.getHeading());
        PathChain shootingToBall1StartPath = lineConstant(shootPose, ball1StartPose, ball1StartPose.getHeading());
        PathChain ball1StartToBall1EndPath = lineConstant(ball1StartPose, ball1EndPose, ball1EndPose.getHeading());
        PathChain ball1EndToShootingPath = lineConstant(ball1EndPose, shootPose, shootPose.getHeading());
        PathChain shootingToBall2StartPath = lineConstant(shootPose, ball2StartPose, ball2StartPose.getHeading());
        PathChain ball2StartToBall2WithGatePath = lineConstant(ball2StartPose, ball2WithGatePose, ball2WithGatePose.getHeading());
        PathChain ball2EndToShootingPath = lineConstant(ball2WithGatePose, shootPose, shootPose.getHeading());
        PathChain shootingToBall3StartPath = lineConstant(shootPose, ball3StartPose, ball3StartPose.getHeading());
        PathChain ball3StartToBall3EndPath = lineConstant(ball3StartPose, ball3EndPose, ball3EndPose.getHeading());
        PathChain ball3EndToShootingPath = lineConstant(ball3EndPose, shootPose, shootPose.getHeading());

        shoot(startToShootingPath);
        intakeAtPos(shootingToBall1StartPath, ball1StartToBall1EndPath);
        shoot(ball1EndToShootingPath);
        intakeAtPos(shootingToBall2StartPath, ball2StartToBall2WithGatePath);
        shoot(ball2EndToShootingPath);
        intakeAtPos(shootingToBall3StartPath, ball3StartToBall3EndPath);
        shoot(ball3EndToShootingPath);
    }

    private PathChain lineConstant(Pose start, Pose end, double heading) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setConstantHeadingInterpolation(heading)
                .build();
    }
}
