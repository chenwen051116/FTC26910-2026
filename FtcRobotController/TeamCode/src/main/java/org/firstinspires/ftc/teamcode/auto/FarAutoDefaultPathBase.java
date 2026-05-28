package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public abstract class FarAutoDefaultPathBase extends FarAutoBase {
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
        FarPoses poses = getFarPoses();

        PathChain startToShootingPath = lineConstant(poses.start, poses.shoot, poses.shoot.getHeading());
        PathChain shootingToBall3StartPath = lineConstant(poses.shoot, poses.ball3Start, poses.ball3Start.getHeading());
        PathChain ball3StartToBall3EndPath = lineConstant(poses.ball3Start, poses.ball3End, poses.ball3End.getHeading());
        PathChain ball3EndToShootingPath = lineConstant(poses.ball3End, poses.shoot, poses.shoot.getHeading());
        PathChain shootingToHumanZoneGetStartPath = lineConstant(poses.shoot, poses.humanZoneGetStart, poses.humanZoneGetStart.getHeading());
        PathChain humanZoneGetStartToHumanZoneGetEndPath = lineConstant(poses.humanZoneGetStart, poses.humanZoneGetEnd, poses.humanZoneGetEnd.getHeading());
        PathChain humanZoneGetEndToShootingPath = lineConstant(poses.humanZoneGetEnd, poses.shoot, poses.shoot.getHeading());
        PathChain shootingToLeavePath = lineConstant(poses.shoot, poses.leave, poses.leave.getHeading());

        shoot(startToShootingPath, startToShootingMaxPower);
        intakeAtPos(shootingToBall3StartPath, ball3StartToBall3EndPath);
        shoot(ball3EndToShootingPath, ball3EndToShootingMaxPower);

        for (int i = 0; i < 2; i++) {
            intakeAtPos(shootingToHumanZoneGetStartPath, humanZoneGetStartToHumanZoneGetEndPath);
            shoot(humanZoneGetEndToShootingPath, humanZoneGetEndToShootingMaxPower);
        }

        goTo(shootingToLeavePath);
    }

    private PathChain lineConstant(Pose start, Pose end, double heading) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setConstantHeadingInterpolation(heading)
                .build();
    }
}
