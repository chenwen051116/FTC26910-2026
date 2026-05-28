package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public abstract class NearAutoDefaultPathBase extends NearAutoBase {
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
        NearPoses poses = getNearPoses();

        PathChain startToShootingPath = lineLinear(poses.start, poses.shoot);
        PathChain shootingToBall2StartPath = lineConstant(poses.shoot, poses.ball2Start, poses.ball2Start.getHeading());
        PathChain ball2StartToBall2EndPath = lineLinear(poses.ball2Start, poses.ball2End);
        PathChain ball2EndToAvoidPath = lineLinear(poses.ball2End, poses.avoid);
        PathChain avoidToShootPath = lineLinear(poses.avoid, poses.shoot);

        PathChain shootingToBeforeGatePath = lineConstant(poses.shoot, poses.beforeGate, poses.beforeGate.getHeading());
        PathChain beforeGateToOpenGatePath = lineConstant(poses.beforeGate, poses.openGate, poses.openGate.getHeading());
        PathChain openGatePoseToAvoidPath = lineLinear(poses.openGate, poses.avoid);

        PathChain shootingToBall1StartPath = lineConstant(poses.shoot, poses.ball1Start, poses.ball1Start.getHeading());
        PathChain ball1StartToBall1EndPath = lineLinear(poses.ball1Start, poses.ball1End);
        PathChain ball1EndToShootingPath = lineConstant(poses.ball1End, poses.shoot, poses.shoot.getHeading());

        PathChain shootingToBall3StartPath = lineConstant(poses.shoot, poses.ball3Start, poses.ball3Start.getHeading());
        PathChain ball3StartToBall3EndPath = lineLinear(poses.ball3Start, poses.ball3End);
        PathChain ball3EndToShootingPath = lineConstant(poses.ball3End, poses.shoot, poses.shoot.getHeading());

        PathChain shootingToLeavePath = lineConstant(poses.shoot, poses.leave, poses.leave.getHeading());

        shoot(startToShootingPath);
        intakeAtPos(shootingToBall2StartPath, ball2StartToBall2EndPath, ball2IntakeMaxPower);
        goTo(ball2EndToAvoidPath, defaultMoveMaxPower, false);
        shoot(avoidToShootPath, avoidToShootingMaxPower);

        for (int i = 0; i < gateCycles(); i++) {
            goTo(shootingToBeforeGatePath);
            intakeToPos(beforeGateToOpenGatePath, defaultIntakeDuration, getGateGetBallPower());
            goTo(openGatePoseToAvoidPath, defaultMoveMaxPower, false);
            shoot(avoidToShootPath);
        }

        intakeAtPos(shootingToBall1StartPath, ball1StartToBall1EndPath);
        shoot(ball1EndToShootingPath);

        if (includeRow3()) {
            intakeAtPos(shootingToBall3StartPath, ball3StartToBall3EndPath);
            shoot(ball3EndToShootingPath);
        }

        goTo(shootingToLeavePath);
    }

    private PathChain lineConstant(Pose start, Pose end, double heading) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setConstantHeadingInterpolation(heading)
                .build();
    }

    private PathChain lineLinear(Pose start, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }
}
