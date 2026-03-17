package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedFar.*;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "Red Far Auto")
public class RedFarAuto extends AutoBase{
    public static double ball3EndToShootingMaxPower = 0.5;
    public static double humanZoneGetEndToShootingMaxPower = 0.5;
    public static double startToShootingMaxPower = 0.5;

    @Override
    public void setStartPose() {
        this.startPose = new Pose(startX, startY, startHeading);
    }

    @Override
    public void setShootPose() {
        this.shootPose = new Pose(shootX, shootY, shootHeading);
    }

    @Override
    public void initializePath() {
        // START POINT
        this.startPose = new Pose(startX, startY, startHeading);

        // SHOOT POINT
        this.shootPose = new Pose(shootX, shootY, shootHeading);

        // BALL 3 START POINT
        Pose ball3StartPose = new Pose(ball3StartX, ball3StartY, ball3StartHeading);

        // BALL 3 END POINT
        Pose ball3EndPose = new Pose(ball3EndX, ball3EndY, ball3EndHeading);

        // HUMAN ZONE START POINT
        Pose humanZoneGetStartPose = new Pose(humanZoneGetStartX, humanZoneGetStartY, humanZoneGetStartHeading);

        // HUMAN ZONE END POINT
        Pose humanZoneGetEndPose = new Pose(humanZoneGetEndX, humanZoneGetEndY, humanZoneGetEndHeading);

        // LEAVE POINT
        Pose leavePose = new Pose(leaveX, leaveY, leaveHeading);

        // FROM START TO SHOOT POINT
        PathChain startToShootingPath = buildPath(startPose, shootPose);

        // FROM SHOOTING ZONE TO BALL 3 START POINT
        PathChain shootingToBall3StartPath = buildPath(shootPose, ball3StartPose);

        // FROM BALL 3 START POINT TO BALL 3 END POINT
        PathChain ball3StartToBall3EndPath = buildPath(ball3StartPose, ball3EndPose);

        // FROM BALL 3 END POINT TO SHOOT POINT
        PathChain ball3EndToShootingPath = buildPath(ball3EndPose, shootPose);

        // FROM SHOOT POINT TO HUMAN ZONE GET POINT START
        PathChain shootingToHumanZoneGetStartPath = buildPath(shootPose, humanZoneGetStartPose);

        // FROM HUMAN ZONE GET POINT START TO HUMAN ZONE GET POINT END
        PathChain humanZoneGetStartToHumanZoneGetEndPath = buildPath(humanZoneGetStartPose, humanZoneGetEndPose);

        // FROM HUMAN ZONE GET POINT END TO SHOOT POINT
        PathChain humanZoneGetEndToShootingPath = buildPath(humanZoneGetEndPose, shootPose);

        // FROM SHOOT POINT TO LEAVE
        PathChain shootingToLeavePath = buildPath(shootPose, leavePose);

        // Start to shoot
        shoot(startToShootingPath, startToShootingMaxPower);

        // Shoot to ball3 intake
        intakeAtPos(shootingToBall3StartPath, ball3StartToBall3EndPath);

        // Ball3 to shoot
        shoot(ball3EndToShootingPath, ball3EndToShootingMaxPower);

        // Shoot to human zone intake
        intakeAtPos(shootingToHumanZoneGetStartPath, humanZoneGetStartToHumanZoneGetEndPath);

        // Human zone to shoot
        shoot(humanZoneGetEndToShootingPath, humanZoneGetEndToShootingMaxPower);

        // Shoot to human zone intake
        intakeAtPos(shootingToHumanZoneGetStartPath, humanZoneGetStartToHumanZoneGetEndPath);

        // Human zone to shoot
        shoot(humanZoneGetEndToShootingPath, humanZoneGetEndToShootingMaxPower);

        // Leave from shoot
        goTo(shootingToLeavePath);
    }
}
