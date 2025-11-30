package org.firstinspires.ftc.teamcode.opmodes; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Scheduler;

@Autonomous(name = "RED_Near_12ball_gate")
public class Red_near_12_balls extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, timer;
    //private final ElapsedTime timer  = new ElapsedTime();

    private int pathState = 0;
    private final Pose startPose = new Pose(0, 0, 0); // Start Pose of our robot.
    private final Pose ShootPose1 = new Pose(-36.53817, -24.4827, 0.83604);
    private final Pose GatePose = new Pose(-0.6099,-34.4572, 1.600);
    private final Pose PrepGather1 = new Pose(-32.0954, -27.4628, 0);

    private final Pose FinishGather1 = new Pose(-2.4513, -27.4628, 0);

    private final Pose PrepGather2 = new Pose(-32.0954, -49.0732, 0);

    private final Pose FinishGather2 = new Pose(-2.4513, -53.0732, 0);

    private final Pose PrepGather3 = new Pose(-32.0954, -70.1802, 0);//accounted for overshoot

    private final Pose FinishGather3 = new Pose(-2.4513, -75.1802, 0);

    private final Pose GatePassby = new Pose(-23.0954, -27.4628, 1.5647);

    private final Pose Park = new Pose(-26.0954, -49.0732, 0.83604);
    private boolean firstshooting = false;
    private PathChain Shootpath1, Shootpath2, Shootpath3,Shootpath4, lastOutPath;
    private PathChain prepGatherPath1, prepGatherPath2, prepGatherPath3;

    public Intake intake;
    public Shooter shooter;
    public Limelight limelight;
    public Scheduler scheduler;

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

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooter.autoLonger = false;
                shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                follower.followPath(Shootpath1,true);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    if (!firstshooting) {
                        shooter.updateFocused(true);
                        shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        timer.resetTimer();

                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()> 3.5){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                            intake.setIntakeState(Intake.IntakeStates.Ball_In);
                            setPathState(2);
                        }
                        if(timer.getElapsedTimeSeconds()>2 && timer.getElapsedTimeSeconds()<3.5){
                        }

                    }
                    break;

                }
            case 2:
                if(!follower.isBusy()) {

                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                    intake.setIntakeState(Intake.IntakeStates.Ball_In);
                    shooter.periodic();
                    follower.followPath(prepGatherPath1);
                    setPathState(3);
                }
                break;
            case 3:
                if(follower.getPose().getX()>-6){
                    intake.setIntakeState(Intake.IntakeStates.Ball_In);
                    intake.periodic();
                }
                else if (follower.getPose().getX() > -9){
                    intake.setIntakeState(Intake.IntakeStates.Send_Ball);
                    intake.periodic();
                }


                if(!follower.isBusy()) {
                    setPathState(4);
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    intake.setIntakeState(Intake.IntakeStates.Ball_In);
                    shooter.periodic();
                }
                break;
            case 4:

                follower.followPath(Shootpath2,true);
                firstshooting = false;
                setPathState(5);

                break;
            case 5:
                if(!follower.isBusy()) {
                    if (!firstshooting) {
                        shooter.updateFocused(true);
                        shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        timer.resetTimer();

                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()> 3.5){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                            intake.setIntakeState(Intake.IntakeStates.Ball_In);
                            setPathState(6);
                        }

                    }
                    break;
                }
            case 6:
                if(!follower.isBusy()) {

                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                    intake.setIntakeState(Intake.IntakeStates.Ball_In);
                    shooter.periodic();
                    follower.followPath(prepGatherPath2);
                    setPathState(7);
                }
                break;
            case 7:
                if(follower.getPose().getX()>-6){
                    intake.setIntakeState(Intake.IntakeStates.Ball_In);
                    intake.periodic();
                }
                else if (follower.getPose().getX() > -9){
                    intake.setIntakeState(Intake.IntakeStates.Send_Ball);
                    intake.periodic();
                }

                if(!follower.isBusy()) {
                    setPathState(8);
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    intake.setIntakeState(Intake.IntakeStates.Ball_In);
                    shooter.periodic();
                }
                break;
            case 8:
                shooter.autoLonger = false;
                follower.followPath(Shootpath3,true);
                firstshooting = false;
                setPathState(9);

                break;
            case 9:
                if(!follower.isBusy()) {
                    if (!firstshooting) {
                        shooter.updateFocused(true);
                        shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        timer.resetTimer();

                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()> 3.5){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                            intake.setIntakeState(Intake.IntakeStates.Ball_In);
                            setPathState(10);
                        }
                        if(timer.getElapsedTimeSeconds()>2 && timer.getElapsedTimeSeconds()<3.5){
                        }

                    }
                    break;
                }
            case 10:
                if(!follower.isBusy()) {

                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                    intake.setIntakeState(Intake.IntakeStates.Ball_In);
                    shooter.periodic();
                    follower.followPath(prepGatherPath3);
                    setPathState(11);
                }
                break;
            case 11:
                if(follower.getPose().getX()>-9){
                    intake.setIntakeState(Intake.IntakeStates.Ball_In);
                    intake.periodic();
                }
                else if (follower.getPose().getX() > -12){
                    intake.setIntakeState(Intake.IntakeStates.Send_Ball);
                    intake.periodic();
                }

                if(!follower.isBusy()) {
                    setPathState(12);
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                    intake.setIntakeState(Intake.IntakeStates.Ball_In);
                    shooter.periodic();
                }
                break;
            case 12:

                follower.followPath(Shootpath4,true);
                firstshooting = false;
                setPathState(13);

                break;
            case 13:
                if(!follower.isBusy()) {
                    if (!firstshooting) {
                        shooter.updateFocused(true);
                        shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                        timer.resetTimer();

                        firstshooting = true;
                    }
                    else{
                        if(timer.getElapsedTimeSeconds()> 3.5){
                            shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                            intake.setIntakeState(Intake.IntakeStates.Ball_In);
                            setPathState(14);
                        }

                    }
                    break;
                }
            case 14:
                if(!follower.isBusy()) {
                    follower.followPath(lastOutPath);
                    setPathState(15);
                }
//                break;
//            case 15:
//                if(!follower.isBusy()) {
//                    setPathState(16);
//                }

        }
    }
    private void sleep(long ms){
        try{
            Thread.sleep(ms);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        intake.periodic();
        shooter.periodic();
        if(shooter.shooterStatus == Shooter.ShooterStatus.Shooting){
            intake.updateAutoShoot(true);
            intake.updateAutoTrans(shooter.isAtTargetRPM());
            shooter.updateDis(limelight.getDis());
            shooter.updateFocused(true);
        }
        else{
            intake.updateAutoShoot(false);
            shooter.updateFocused(false);
        }
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("realRPM", shooter.getFlyWheelRPM());
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("timer", timer.getElapsedTimeSeconds());
        telemetry.addData("shooter state", shooter.shooterStatus);
        telemetry.addData("intake state", intake.intakeCurrentState);
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        timer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shooter.automode = true;
        limelight = new Limelight(hardwareMap);
        limelight.initRedPipeline();
        limelight.startDetect();
        intake.setIntakeState(Intake.IntakeStates.Stop);
        shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
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

    public void setScheduler(Scheduler scheduler) {
        this.scheduler = scheduler;
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}