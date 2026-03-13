package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "New red Auto test")
public class TestRedAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, timer;
    //private final ElapsedTime timer  = new ElapsedTime();

    private int pathState = 0;
    private final Pose startPose = new Pose(0, 0, 0); // Start Pose of our robot.
    private final Pose ShootPose1 = new Pose(-28.53817, -16.4827, 0.63604);
    private final Pose GatePose = new Pose(-2.6099,-33.9572, 1.600);
    private final Pose PrepGather1 = new Pose(-30.0954, -26.9628, 0);

    private final Pose FinishGather1 = new Pose(-6.4513, -26.9628, 0);

    private final Pose PrepGather2 = new Pose(-30.0954, -48.5732, 0);

    private final Pose FinishGather2 = new Pose(-6.4513, -48.5732, 0);

    private final Pose PrepGather3 = new Pose(-30.0954, -68.6802, 0);//accounted for overshoot

    private final Pose FinishGather3 = new Pose(-6.4513, -68.6802, 0);

    private final Pose GatePassby = new Pose(-23.0954, -30.9628, 1.5647);

    private final Pose Park = new Pose(-26.0954, -52.5732, 0.83604);
    private boolean shooting = false;
    private PathChain Shootpath1, Shootpath2, Shootpath3,Shootpath4, lastOutPath;
    private PathChain prepGatherPath1, prepGatherPath2, prepGatherPath3;

    public void buildPaths() {

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Shootpath1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, ShootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), ShootPose1.getHeading())

                .build();

        prepGatherPath1 = follower.pathBuilder()
                .addPath(new BezierLine(ShootPose1, PrepGather1))
                .setLinearHeadingInterpolation(ShootPose1.getHeading(), PrepGather1.getHeading())
                .addPath(new BezierLine(PrepGather1, FinishGather1))
                .setLinearHeadingInterpolation(PrepGather1.getHeading(), FinishGather1.getHeading())
                .addPath(new BezierLine(FinishGather1, GatePose))
                .setLinearHeadingInterpolation(FinishGather1.getHeading(), GatePose.getHeading())
                .build();



        Shootpath2 = follower.pathBuilder()
                .addPath(new BezierLine(GatePose, GatePassby))
                .setLinearHeadingInterpolation(GatePose.getHeading(), GatePassby.getHeading())
                .addPath(new BezierLine(GatePassby, ShootPose1))
                .setLinearHeadingInterpolation(GatePassby.getHeading(), ShootPose1.getHeading())
                .build();

        prepGatherPath2 = follower.pathBuilder()
                .addPath(new BezierLine(ShootPose1, PrepGather2))
                .setLinearHeadingInterpolation(ShootPose1.getHeading(), PrepGather2.getHeading())
                .addPath(new BezierLine(PrepGather2, FinishGather2))
                .setLinearHeadingInterpolation(PrepGather2.getHeading(), FinishGather2.getHeading())
                .build();

        Shootpath3 = follower.pathBuilder()

                .addPath(new BezierLine(FinishGather2, ShootPose1))
                .setLinearHeadingInterpolation(PrepGather2.getHeading(), ShootPose1.getHeading())
                .build();

        prepGatherPath3 = follower.pathBuilder()
                .addPath(new BezierLine(ShootPose1, PrepGather3))
                .setLinearHeadingInterpolation(ShootPose1.getHeading(), PrepGather3.getHeading())
                .addPath(new BezierLine(PrepGather3, FinishGather3))
                .setLinearHeadingInterpolation(PrepGather3.getHeading(), FinishGather3.getHeading())
                .setBrakingStrength(1.5)
                .build();

        Shootpath4 = follower.pathBuilder()

                .addPath(new BezierLine(FinishGather3, ShootPose1))
                .setLinearHeadingInterpolation(PrepGather3.getHeading(), ShootPose1.getHeading())
                .build();
        lastOutPath = follower.pathBuilder()

                .addPath(new BezierLine(ShootPose1, Park))
                .setLinearHeadingInterpolation(ShootPose1.getHeading(), Park.getHeading())
                .build();
//
//        lastOutPath = follower.pathBuilder()
//                .addPath(new BezierLine(ShootPose1, endPose))
//                .setLinearHeadingInterpolation(ShootPose1.getHeading(), endPose.getHeading())
//                .build();

    }


    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void autonomousPathUpdate(){
        double prevDis;
        switch (pathState){
            case 0: // Path: From initial position to shooting position
                follower.followPath(Shootpath1, true);
                setPathState(1);
                prevDis = 0;
                break;

            case 1: // Shooting
                if (!follower.isBusy()){
                    if (!shooting){
                        shooting = true;
                        timer.resetTimer();
                    }
                    else {
                        if (timer.getElapsedTimeSeconds() > 3.5){
                            setPathState(2);
                        }
                    }
                    break;
                }

            case 2: // Path: From shooting position to prep gather position 1
                if(!follower.isBusy()) {
                    follower.followPath(prepGatherPath1);
                    setPathState(3);
                }
                break;

            case 3: // Path: Gather ball 1
                if(!follower.isBusy()) {
                    setPathState(4);
                }
                break;

            case 4: // Path: From gather ball position 1 back to shooting position
                follower.followPath(Shootpath2,true);
                shooting = false;
                setPathState(5);
                break;

            case 5: // Shooting 2
                if (!follower.isBusy()){
                    if (!shooting){
                        shooting = true;
                        timer.resetTimer();
                    }
                    else {
                        if (timer.getElapsedTimeSeconds() > 3.5){
                            setPathState(6);
                        }
                    }
                    break;
                }

            case 6: // Path: From Shooting position to prep gather position 2
                if(!follower.isBusy()) {
                    follower.followPath(prepGatherPath2);
                    setPathState(7);
                }
                break;

            case 7: // Path: Gather ball 2
                if(!follower.isBusy()) {
                    setPathState(8);
                }
                break;

            case 8: // Path: From gather ball position 2 back to shooting position
                follower.followPath(Shootpath3,true);
                shooting = false;
                setPathState(9);
                break;

            case 9: // Shooting 3
                if (!follower.isBusy()){
                    if (!shooting){
                        shooting = true;
                        timer.resetTimer();
                    }
                    else {
                        if (timer.getElapsedTimeSeconds() > 3.5){
                            setPathState(10);
                        }
                    }
                    break;
                }

            case 10: // Path: From Shooting position to prep gather position 3
                if(!follower.isBusy()) {
                    follower.followPath(prepGatherPath3);
                    setPathState(11);
                }
                break;

            case 11: // Path: Gather ball 3
                if(!follower.isBusy()) {
                    setPathState(12);
                }
                break;

            case 12: // Path: From gather ball position 3 back to shooting position
                follower.followPath(Shootpath4,true);
                shooting = false;
                setPathState(13);
                break;

            case 13: // Shooting 4
                if (!follower.isBusy()){
                    if (!shooting){
                        shooting = true;
                        timer.resetTimer();
                    }
                    else {
                        if (timer.getElapsedTimeSeconds() > 3.5){
                            setPathState(14);
                        }
                        else{
                        }
                    }
                    break;
                }

            case 14:

                if(!follower.isBusy()) {
                    follower.followPath(lastOutPath);
                    setPathState(15);
                }
        }
    }
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        shooting = false;
        pathTimer = new Timer();
        timer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }


    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}