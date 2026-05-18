package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.BezierLine;
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

        PathChain startToShootingPath = follower.pathBuilder()
                .addPath(new BezierLine(poses.start, poses.shoot))
                .setLinearHeadingInterpolation(poses.start.getHeading(), poses.shoot.getHeading())
                .setTValueConstraint(shootingTValue)
                .setBrakingStrength(defaultBrakingStrength)
                .setBrakingStart(startToShootingBrakingStart)
                .setGlobalDeceleration(defaultBrakingStrength)
                .build();

        PathChain shootingToBall2StartPath = follower.pathBuilder()
                .addPath(new BezierLine(poses.shoot, poses.ball2Start))
                .setConstantHeadingInterpolation(poses.ball2Start.getHeading())
                .setTValueConstraint(0.997)
                .setBrakingStrength(ball2IntakeBrakingStrength)
                .setBrakingStart(ball2IntakeBrakingStart)
                .setGlobalDeceleration(ball2IntakeBrakingStrength)
                .build();
        PathChain ball2StartToBall2EndPath = follower.pathBuilder()
                .addPath(new BezierLine(poses.ball2Start, poses.ball2End))
                .setLinearHeadingInterpolation(poses.ball2Start.getHeading(), poses.ball2End.getHeading())
                .setTValueConstraint(0.997)
                .setBrakingStrength(defaultBrakingStrength)
                .setBrakingStart(defaultBrakingStart)
                .setGlobalDeceleration(defaultBrakingStrength)
                .build();
        PathChain ball2EndToAvoidPath = follower.pathBuilder()
                .addPath(new BezierLine(poses.ball2End, poses.avoid))
                .setLinearHeadingInterpolation(poses.ball2End.getHeading(), poses.avoid.getHeading())
                .setTValueConstraint(avoidTValue)
                .build();
        PathChain avoidToShootPath = follower.pathBuilder()
                .addPath(new BezierLine(poses.avoid, poses.shoot))
                .setLinearHeadingInterpolation(poses.avoid.getHeading(), poses.shoot.getHeading())
                .setTValueConstraint(0.997)
                .setBrakingStrength(avoidToShootingBrakingStrength)
                .setBrakingStart(avoidToShootingBrakingStart)
                .setGlobalDeceleration(avoidToShootingBrakingStrength)
                .build();

        PathChain shootingToBeforeGatePath = follower.pathBuilder()
                .addPath(new BezierLine(poses.shoot, poses.beforeGate))
                .setConstantHeadingInterpolation(poses.beforeGate.getHeading())
                .setTValueConstraint(0.997)
                .setBrakingStrength(defaultBrakingStrength)
                .setBrakingStart(defaultBrakingStart)
                .setGlobalDeceleration(defaultBrakingStrength)
                .build();
        PathChain beforeGateToOpenGatePath = follower.pathBuilder()
                .addPath(new BezierLine(poses.beforeGate, poses.openGate))
                .setConstantHeadingInterpolation(poses.openGate.getHeading())
                .setTValueConstraint(gateTValue)
                .setBrakingStrength(defaultBrakingStrength)
                .setBrakingStart(defaultBrakingStart)
                .setGlobalDeceleration(defaultBrakingStrength)
                .build();
        PathChain openGatePoseToAvoidPath = follower.pathBuilder()
                .addPath(new BezierLine(poses.openGate, poses.avoid))
                .setLinearHeadingInterpolation(poses.openGate.getHeading(), poses.avoid.getHeading())
                .setTValueConstraint(avoidTValue)
                .build();

        PathChain shootingToBall1StartPath = follower.pathBuilder()
                .addPath(new BezierLine(poses.shoot, poses.ball1Start))
                .setConstantHeadingInterpolation(poses.ball1Start.getHeading())
                .setTValueConstraint(0.997)
                .setBrakingStrength(defaultBrakingStrength)
                .setBrakingStart(defaultBrakingStart)
                .setGlobalDeceleration(defaultBrakingStrength)
                .build();
        PathChain ball1StartToBall1EndPath = follower.pathBuilder()
                .addPath(new BezierLine(poses.ball1Start, poses.ball1End))
                .setLinearHeadingInterpolation(poses.ball1Start.getHeading(), poses.ball1End.getHeading())
                .setTValueConstraint(0.997)
                .setBrakingStrength(defaultBrakingStrength)
                .setBrakingStart(defaultBrakingStart)
                .setGlobalDeceleration(defaultBrakingStrength)
                .build();
        PathChain ball1EndToShootingPath = follower.pathBuilder()
                .addPath(new BezierLine(poses.ball1End, poses.shoot))
                .setConstantHeadingInterpolation(poses.shoot.getHeading())
                .setTValueConstraint(0.997)
                .setBrakingStrength(defaultBrakingStrength)
                .setBrakingStart(ball1EndToShootingBrakingStart)
                .setGlobalDeceleration(defaultBrakingStrength)
                .build();

        PathChain shootingToBall3StartPath = follower.pathBuilder()
                .addPath(new BezierLine(poses.shoot, poses.ball3Start))
                .setConstantHeadingInterpolation(poses.ball3Start.getHeading())
                .setTValueConstraint(0.997)
                .setBrakingStrength(defaultBrakingStrength)
                .setBrakingStart(ball3IntakeBrakingStart)
                .setGlobalDeceleration(defaultBrakingStrength)
                .build();
        PathChain ball3StartToBall3EndPath = follower.pathBuilder()
                .addPath(new BezierLine(poses.ball3Start, poses.ball3End))
                .setLinearHeadingInterpolation(poses.ball3Start.getHeading(), poses.ball3End.getHeading())
                .setTValueConstraint(0.997)
                .setBrakingStrength(defaultBrakingStrength)
                .setBrakingStart(defaultBrakingStart)
                .setGlobalDeceleration(defaultBrakingStrength)
                .build();
        PathChain ball3EndToShootingPath = follower.pathBuilder()
                .addPath(new BezierLine(poses.ball3End, poses.shoot))
                .setConstantHeadingInterpolation(poses.shoot.getHeading())
                .setTValueConstraint(0.997)
                .setBrakingStrength(ball3EndToShootingBrakingStrength)
                .setBrakingStart(ball3EndToShootingBrakingStart)
                .setGlobalDeceleration(ball3EndToShootingBrakingStrength)
                .build();

        PathChain shootingToLeavePath = follower.pathBuilder()
                .addPath(new BezierLine(poses.shoot, poses.leave))
                .setConstantHeadingInterpolation(poses.leave.getHeading())
                .setTValueConstraint(0.997)
                .setBrakingStrength(defaultBrakingStrength)
                .setBrakingStart(defaultBrakingStart)
                .setGlobalDeceleration(defaultBrakingStrength)
                .build();

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
