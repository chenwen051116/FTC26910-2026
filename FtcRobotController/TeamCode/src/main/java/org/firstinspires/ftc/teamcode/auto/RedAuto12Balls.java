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

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red 12 Balls")
public class RedAuto12Balls extends AutoBase{
    PathChain startToShootingPath,
            shootingToBall1StartPath,
            ball1StartToBall1EndPath,
            ball1EndToShootingPath,
            shootingToBall2StartPath,
            ball2StartToBall2EndPath,
            ball2EndToShootingPath,
            shootingToBall3StartPath,
            ball3StartToBall3EndPath,
            ball3EndToShootingPath;

    @Override
    public void buildPath() {
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
        Pose ball2EndPose = new Pose(ball2EndX, ball2EndY, ball2EndHeading);

        // BALL 3 START POINT
        Pose ball3StartPose = new Pose(ball3StartX, ball3StartY, ball3StartHeading);

        // BALL 3 END POINT
        Pose ball3EndPose = new Pose(ball3EndX, ball3EndY, ball3EndHeading);


        // FROM START TO SHOOTING
        startToShootingPath = buildShootingPath(startPose, shootPose);

        // FROM SHOOTING TO BALL 1 BEGIN
        shootingToBall1StartPath = buildIntakePath(shootPose, ball1StartPose);

        // FROM BALL 1 BEGIN TO BALL 1 END
        ball1StartToBall1EndPath = buildIntakePath(ball1StartPose, ball1EndPose);

        // FROM BALL 1 END TO SHOOTING
        ball1EndToShootingPath = buildShootingPath(ball1EndPose, shootPose);

        // FROM SHOOTING TO BALL 2 BEGIN
        shootingToBall2StartPath = buildIntakePath(shootPose, ball2StartPose);

        // FROM BALL 2 BEGIN TO BALL 2 END
        ball2StartToBall2EndPath = buildIntakePath(ball2StartPose, ball2EndPose);

        // FROM BALL 2 END TO SHOOTING
        ball2EndToShootingPath = buildShootingPath(ball2EndPose, shootPose);

        // FROM SHOOTING TO BALL 3 BEGIN
        shootingToBall3StartPath = buildIntakePath(shootPose, ball3StartPose);

        // FROM BALL 3 BEGIN TO BALL 3 END
        ball3StartToBall3EndPath = buildIntakePath(ball3StartPose, ball3EndPose);

        // FROM BALL 3 END TO SHOOTING
        ball3EndToShootingPath = buildShootingPath(ball3EndPose, shootPose);
    }

    @Override
    public void updatePath() {
        switch (this.pathState) {
            case 1: // FROM START TO SHOOTING
                follow(startToShootingPath);
                break;

            case 2:
                shoot();
                break;

            case 3:
                follow(shootingToBall1StartPath);
                break;

            case 4:
                intake(ball1StartToBall1EndPath);
                break;

            case 5:
                follow(ball1EndToShootingPath);
                break;

            case 6:
                shoot();
                break;

            case 7:
                follow(shootingToBall2StartPath);
                break;

            case 8:
                intake(ball2StartToBall2EndPath);
                break;

            case 9:
                follow(ball2EndToShootingPath);
                break;

            case 10:
                shoot();
                break;

            case 11:
                follow(shootingToBall3StartPath);
                break;

            case 12:
                intake(ball3StartToBall3EndPath);
                break;

            case 13:
                follow(ball3EndToShootingPath);
                break;

            case 14:
                shoot();
                break;
        }
    }
}
