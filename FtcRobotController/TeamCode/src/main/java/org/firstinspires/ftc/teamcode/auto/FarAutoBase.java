package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public abstract class FarAutoBase extends AutoBase {
    public static double ball3EndToShootingMaxPower = 0.5;
    public static double humanZoneGetEndToShootingMaxPower = 0.5;
    public static double startToShootingMaxPower = 0.5;

    protected static class FarPoses {
        final Pose start;
        final Pose shoot;
        final Pose ball3Start;
        final Pose ball3End;
        final Pose humanZoneGetStart;
        final Pose humanZoneGetEnd;
        final Pose leave;

        FarPoses(
                Pose start,
                Pose shoot,
                Pose ball3Start,
                Pose ball3End,
                Pose humanZoneGetStart,
                Pose humanZoneGetEnd,
                Pose leave
        ) {
            this.start = start;
            this.shoot = shoot;
            this.ball3Start = ball3Start;
            this.ball3End = ball3End;
            this.humanZoneGetStart = humanZoneGetStart;
            this.humanZoneGetEnd = humanZoneGetEnd;
            this.leave = leave;
        }
    }

    protected abstract FarPoses getFarPoses();

    @Override
    public void setStartPose() {
        startPose = getFarPoses().start;
    }

    @Override
    public void setShootPose() {
        shootPose = getFarPoses().shoot;
    }

    @Override
    public void initializePath() {
        FarPoses poses = getFarPoses();

        PathChain startToShootingPath = buildPath(poses.start, poses.shoot);
        PathChain shootingToBall3StartPath = buildPath(poses.shoot, poses.ball3Start);
        PathChain ball3StartToBall3EndPath = buildPath(poses.ball3Start, poses.ball3End);
        PathChain ball3EndToShootingPath = buildPath(poses.ball3End, poses.shoot);
        PathChain shootingToHumanZoneGetStartPath = buildPath(poses.shoot, poses.humanZoneGetStart);
        PathChain humanZoneGetStartToHumanZoneGetEndPath = buildPath(
                poses.humanZoneGetStart,
                poses.humanZoneGetEnd
        );
        PathChain humanZoneGetEndToShootingPath = buildPath(poses.humanZoneGetEnd, poses.shoot);
        PathChain shootingToLeavePath = buildPath(poses.shoot, poses.leave);

        shoot(startToShootingPath, startToShootingMaxPower);
        intakeAtPos(shootingToBall3StartPath, ball3StartToBall3EndPath);
        shoot(ball3EndToShootingPath, ball3EndToShootingMaxPower);

        for (int i = 0; i < 2; i++) {
            intakeAtPos(shootingToHumanZoneGetStartPath, humanZoneGetStartToHumanZoneGetEndPath);
            shoot(humanZoneGetEndToShootingPath, humanZoneGetEndToShootingMaxPower);
        }

        goTo(shootingToLeavePath);
    }
}
