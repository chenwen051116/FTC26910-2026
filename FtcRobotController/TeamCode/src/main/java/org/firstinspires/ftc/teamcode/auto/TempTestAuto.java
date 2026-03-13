package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball1EndHeading;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball1EndX;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball1EndY;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball1StartHeading;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball1StartX;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball1StartY;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball2EndHeading;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball2EndX;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball2EndY;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball2StartHeading;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball2StartX;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball2StartY;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball3EndHeading;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball3EndX;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball3EndY;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball3StartHeading;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball3StartX;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.ball3StartY;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.shootHeading;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.shootX;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.shootY;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.startHeading;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.startX;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.startY;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "new temp auto")
public class TempTestAuto extends OpMode {
    private Follower follower;

    public PathChain buildPath(Pose startPose,
                               Pose endPose) {
        return follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setConstantHeadingInterpolation(endPose.getHeading())
                .setTValueConstraint(0.997)
                .build();
    }

    public PathChain buildIntakePath(Pose startPose, Pose endPose) {
        return buildPath(startPose,
                endPose
        );
    }

    public PathChain buildShootingPath(Pose startPose, Pose endPose) {
        return buildPath(startPose,
                endPose
        );
    }
    PathChain startToShootingPath;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);



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

        // BALL 3 START POINT
        Pose ball3StartPose = new Pose(ball3StartX, ball3StartY, ball3StartHeading);

        // BALL 3 END POINT
        Pose ball3EndPose = new Pose(ball3EndX, ball3EndY, ball3EndHeading);



        // FROM START TO SHOOTING
        startToShootingPath = buildShootingPath(startPose, shootPose);

        // FROM SHOOTING TO BALL 1 BEGIN
        PathChain shootingToBall1StartPath = buildIntakePath(shootPose, ball1StartPose);

        // FROM BALL 1 BEGIN TO BALL 1 END
        PathChain ball1StartToBall1EndPath = buildIntakePath(ball1StartPose, ball1EndPose);

        // FROM BALL 1 END TO SHOOTING
        PathChain ball1EndToShootingPath = buildShootingPath(ball1EndPose, shootPose);

        // FROM SHOOTING TO BALL 2 BEGIN
        PathChain shootingToBall2StartPath = buildIntakePath(shootPose, ball2StartPose);

        // FROM BALL 2 BEGIN TO BALL 2 END
        PathChain ball2StartToBall2EndPath = buildIntakePath(ball2StartPose, ball2EndPose);

        // FROM BALL 2 END TO SHOOTING
        PathChain ball2EndToShootingPath = buildShootingPath(ball2EndPose, shootPose);

        // FROM SHOOTING TO BALL 3 BEGIN
        PathChain shootingToBall3StartPath = buildIntakePath(shootPose, ball3StartPose);

        // FROM BALL 3 BEGIN TO BALL 3 END
        PathChain ball3StartToBall3EndPath = buildIntakePath(ball3StartPose, ball3EndPose);

        // FROM BALL 3 END TO SHOOTING

        follower.setStartingPose(startPose);

    }

    @Override
    public void start() {
        follower.followPath(startToShootingPath);
    }

    @Override
    public void loop() {
        follower.update();
    }
}
