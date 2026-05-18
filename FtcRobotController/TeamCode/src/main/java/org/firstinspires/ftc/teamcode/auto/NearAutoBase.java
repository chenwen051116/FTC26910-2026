package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public abstract class NearAutoBase extends AutoBase {
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

    protected static class NearPoses {
        final Pose start;
        final Pose shoot;
        final Pose ball1Start;
        final Pose ball1End;
        final Pose ball2Start;
        final Pose ball2End;
        final Pose ball3Start;
        final Pose ball3End;
        final Pose beforeGate;
        final Pose openGate;
        final Pose leave;
        final Pose avoid;

        NearPoses(
                Pose start,
                Pose shoot,
                Pose ball1Start,
                Pose ball1End,
                Pose ball2Start,
                Pose ball2End,
                Pose ball3Start,
                Pose ball3End,
                Pose beforeGate,
                Pose openGate,
                Pose leave,
                Pose avoid
        ) {
            this.start = start;
            this.shoot = shoot;
            this.ball1Start = ball1Start;
            this.ball1End = ball1End;
            this.ball2Start = ball2Start;
            this.ball2End = ball2End;
            this.ball3Start = ball3Start;
            this.ball3End = ball3End;
            this.beforeGate = beforeGate;
            this.openGate = openGate;
            this.leave = leave;
            this.avoid = avoid;
        }
    }

    protected abstract NearPoses getNearPoses();

    protected boolean includeRow3() {
        return true;
    }

    protected int gateCycles() {
        return includeRow3() ? 1 : 2;
    }

    protected double getGateGetBallPower() {
        return gateGetBallPower;
    }

    @Override
    public void setStartPose() {
        startPose = getNearPoses().start;
    }

    @Override
    public void setShootPose() {
        shootPose = getNearPoses().shoot;
    }

    @Override
    public void initializePath() {
        NearPoses poses = getNearPoses();

        PathChain startToShootingPath = buildPath(
                poses.start,
                poses.shoot,
                defaultBrakingStrength,
                startToShootingBrakingStart,
                shootingTValue,
                HeadingInterpolation.LINEAR
        );

        PathChain shootingToBall2StartPath = buildPath(
                poses.shoot,
                poses.ball2Start,
                ball2IntakeBrakingStrength,
                ball2IntakeBrakingStart
        );
        PathChain ball2StartToBall2EndPath = buildPath(
                poses.ball2Start,
                poses.ball2End,
                HeadingInterpolation.LINEAR
        );
        PathChain ball2EndToAvoidPath = buildPath(
                poses.ball2End,
                poses.avoid,
                avoidTValue,
                HeadingInterpolation.LINEAR,
                false
        );
        PathChain avoidToShootPath = buildPath(
                poses.avoid,
                poses.shoot,
                avoidToShootingBrakingStrength,
                avoidToShootingBrakingStart,
                HeadingInterpolation.LINEAR
        );

        PathChain shootingToBeforeGatePath = buildPath(poses.shoot, poses.beforeGate);
        PathChain beforeGateToOpenGatePath = buildPath(
                poses.beforeGate,
                poses.openGate,
                defaultBrakingStrength,
                defaultBrakingStart,
                gateTValue
        );
        PathChain openGatePoseToAvoidPath = buildPath(
                poses.openGate,
                poses.avoid,
                avoidTValue,
                HeadingInterpolation.LINEAR,
                false
        );

        PathChain shootingToBall1StartPath = buildPath(poses.shoot, poses.ball1Start);
        PathChain ball1StartToBall1EndPath = buildPath(
                poses.ball1Start,
                poses.ball1End,
                HeadingInterpolation.LINEAR
        );
        PathChain ball1EndToShootingPath = buildPath(
                poses.ball1End,
                poses.shoot,
                defaultBrakingStrength,
                ball1EndToShootingBrakingStart
        );

        PathChain shootingToBall3StartPath = buildPath(
                poses.shoot,
                poses.ball3Start,
                defaultBrakingStrength,
                ball3IntakeBrakingStart
        );
        PathChain ball3StartToBall3EndPath = buildPath(
                poses.ball3Start,
                poses.ball3End,
                HeadingInterpolation.LINEAR
        );
        PathChain ball3EndToShootingPath = buildPath(
                poses.ball3End,
                poses.shoot,
                ball3EndToShootingBrakingStrength,
                ball3EndToShootingBrakingStart
        );

        PathChain shootingToLeavePath = buildPath(poses.shoot, poses.leave);

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
}
