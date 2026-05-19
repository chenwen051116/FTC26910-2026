package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;

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

        PathChain startToShootingPath = follower.pathBuilder()
                .addPath(new BezierLine(poses.start, poses.shoot))
                .setConstantHeadingInterpolation(poses.shoot.getHeading())
                .setGlobalDeceleration()
                .build();
        startToShootingPath.setConstraintsForAll(new PathConstraints(0.997, 100, defaultBrakingStrength, defaultBrakingStart));
        PathChain shootingToBall3StartPath = follower.pathBuilder()
                .addPath(new BezierLine(poses.shoot, poses.ball3Start))
                .setConstantHeadingInterpolation(poses.ball3Start.getHeading())
                .setGlobalDeceleration()
                .build();
        shootingToBall3StartPath.setConstraintsForAll(new PathConstraints(0.997, 100, defaultBrakingStrength, defaultBrakingStart));
        PathChain ball3StartToBall3EndPath = follower.pathBuilder()
                .addPath(new BezierLine(poses.ball3Start, poses.ball3End))
                .setConstantHeadingInterpolation(poses.ball3End.getHeading())
                .setGlobalDeceleration()
                .build();
        ball3StartToBall3EndPath.setConstraintsForAll(new PathConstraints(0.997, 100, defaultBrakingStrength, defaultBrakingStart));
        PathChain ball3EndToShootingPath = follower.pathBuilder()
                .addPath(new BezierLine(poses.ball3End, poses.shoot))
                .setConstantHeadingInterpolation(poses.shoot.getHeading())
                .setGlobalDeceleration()
                .build();
        ball3EndToShootingPath.setConstraintsForAll(new PathConstraints(0.997, 100, defaultBrakingStrength, defaultBrakingStart));
        PathChain shootingToHumanZoneGetStartPath = follower.pathBuilder()
                .addPath(new BezierLine(poses.shoot, poses.humanZoneGetStart))
                .setConstantHeadingInterpolation(poses.humanZoneGetStart.getHeading())
                .setGlobalDeceleration()
                .build();
        shootingToHumanZoneGetStartPath.setConstraintsForAll(new PathConstraints(0.997, 100, defaultBrakingStrength, defaultBrakingStart));
        PathChain humanZoneGetStartToHumanZoneGetEndPath = follower.pathBuilder()
                .addPath(new BezierLine(poses.humanZoneGetStart, poses.humanZoneGetEnd))
                .setConstantHeadingInterpolation(poses.humanZoneGetEnd.getHeading())
                .setGlobalDeceleration()
                .build();
        humanZoneGetStartToHumanZoneGetEndPath.setConstraintsForAll(new PathConstraints(0.997, 100, defaultBrakingStrength, defaultBrakingStart));
        PathChain humanZoneGetEndToShootingPath = follower.pathBuilder()
                .addPath(new BezierLine(poses.humanZoneGetEnd, poses.shoot))
                .setConstantHeadingInterpolation(poses.shoot.getHeading())
                .setGlobalDeceleration()
                .build();
        humanZoneGetEndToShootingPath.setConstraintsForAll(new PathConstraints(0.997, 100, defaultBrakingStrength, defaultBrakingStart));
        PathChain shootingToLeavePath = follower.pathBuilder()
                .addPath(new BezierLine(poses.shoot, poses.leave))
                .setConstantHeadingInterpolation(poses.leave.getHeading())
                .setGlobalDeceleration()
                .build();
        shootingToLeavePath.setConstraintsForAll(new PathConstraints(0.997, 100, defaultBrakingStrength, defaultBrakingStart));

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
