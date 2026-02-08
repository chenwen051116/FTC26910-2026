package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subSystems.DistSensor;
import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.Limelight;
import org.firstinspires.ftc.teamcode.subSystems.Scheduler;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.Shooter.ShooterStates;
import org.firstinspires.ftc.teamcode.subSystems.Turret;
import org.firstinspires.ftc.teamcode.subSystems.Turret.TurretShooterStates;

@Autonomous(name = "New red Auto test")
public class RedNewAuto extends OpMode {

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

    public Intake intake;
    public Shooter shooter;
    public Limelight limelight;
    public DistSensor distSensor;
    public Scheduler scheduler;
    public Turret turret;
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
                shooter.setShooterStatusTo(ShooterStates.Idling);
                follower.followPath(Shootpath1, true);
                turret.setCurrentPosTo(intake.getEncoderValue());
                setPathState(1);
                prevDis = 0;
                break;

            case 1: // Shooting
                if (!follower.isBusy()){
                    if (!shooting){
                        shooting = true;
                        shooter.setShooterStatusTo(ShooterStates.Shooting);
                        turret.tx = limelight.getTx();
                        turret.setCurrentPosTo(intake.getEncoderValue());
                        turret.setShooterStatusTo(TurretShooterStates.Shooting);
                        intake.setIntakeStatusTo(Intake.IntakeStates.Stop);
                        intake.setShooterStatusTo(Intake.IntakeShooterStates.Shooting);

                        timer.resetTimer();
                    }
                    else {
                        if (timer.getElapsedTimeSeconds() > 3.5){
                            shooter.setShooterStatusTo(ShooterStates.Stop);
                            turret.setShooterStatusTo(TurretShooterStates.Off);
                            turret.setCurrentPosTo(intake.getEncoderValue());
                            intake.setIntakeStatusTo(Intake.IntakeStates.Intake);
                            setPathState(2);
                        }
                        else{
                            turret.tx = limelight.getTx();
                            turret.setCurrentPosTo(intake.getEncoderValue());
                            intake.setShooterIsAtTargetRPMStatusTo(shooter.isAtTargetRPM());
                            intake.setIntakeStatusTo(Intake.IntakeStates.Stop);
                        }
                    }
                    break;
                }

            case 2: // Path: From shooting position to prep gather position 1
                turret.setShooterStatusTo(TurretShooterStates.Off);
                turret.setCurrentPosTo(intake.getEncoderValue());
                if(!follower.isBusy()) {
                    shooter.setShooterStatusTo(Shooter.ShooterStates.Stop);
                    intake.setIntakeStatusTo(Intake.IntakeStates.Intake);
                    shooter.periodic();
                    follower.followPath(prepGatherPath1);
                    setPathState(3);
                }
                break;

            case 3: // Path: Gather ball 1
                turret.setShooterStatusTo(TurretShooterStates.Off);
                turret.setCurrentPosTo(intake.getEncoderValue());
                intake.setFirstBallStatusTo(!distSensor.containsFirstBall());
                if(!follower.isBusy()) {
                    setPathState(4);
                    shooter.setShooterStatusTo(Shooter.ShooterStates.Idling);
                    intake.setIntakeStatusTo(Intake.IntakeStates.Intake);
                    shooter.periodic();
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
                        shooter.setShooterStatusTo(ShooterStates.Shooting);
                        turret.tx = limelight.getTx();
                        turret.setCurrentPosTo(intake.getEncoderValue());
                        turret.setShooterStatusTo(TurretShooterStates.Shooting);
                        intake.setIntakeStatusTo(Intake.IntakeStates.Stop);
                        intake.setShooterStatusTo(Intake.IntakeShooterStates.Shooting);
                        intake.setShooterIsAtTargetRPMStatusTo(shooter.isAtTargetRPM());
                        timer.resetTimer();
                    }
                    else {
                        if (timer.getElapsedTimeSeconds() > 3.5){
                            shooter.setShooterStatusTo(ShooterStates.Stop);
                            turret.setShooterStatusTo(TurretShooterStates.Off);
                            turret.setCurrentPosTo(intake.getEncoderValue());
                            intake.setIntakeStatusTo(Intake.IntakeStates.Intake);
                            setPathState(6);
                        }
                        else{
                            turret.tx = limelight.getTx();
                            turret.setCurrentPosTo(intake.getEncoderValue());
                            intake.setShooterIsAtTargetRPMStatusTo(shooter.isAtTargetRPM());
                            intake.setIntakeStatusTo(Intake.IntakeStates.Stop);
                        }
                    }
                    break;
                }

            case 6: // Path: From Shooting position to prep gather position 2
                turret.setShooterStatusTo(TurretShooterStates.Off);
                turret.setCurrentPosTo(intake.getEncoderValue());
                if(!follower.isBusy()) {
                    shooter.setShooterStatusTo(Shooter.ShooterStates.Stop);
                    intake.setIntakeStatusTo(Intake.IntakeStates.Intake);
                    shooter.periodic();
                    follower.followPath(prepGatherPath2);
                    setPathState(7);
                }
                break;

            case 7: // Path: Gather ball 2
                turret.setShooterStatusTo(TurretShooterStates.Off);
                turret.setCurrentPosTo(intake.getEncoderValue());
                intake.setFirstBallStatusTo(!distSensor.containsFirstBall());
                if(!follower.isBusy()) {
                    setPathState(8);
                    shooter.setShooterStatusTo(Shooter.ShooterStates.Idling);
                    intake.setIntakeStatusTo(Intake.IntakeStates.Intake);
                    shooter.periodic();
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
                        shooter.setShooterStatusTo(ShooterStates.Shooting);
                        turret.tx = limelight.getTx();
                        turret.setCurrentPosTo(intake.getEncoderValue());
                        turret.setShooterStatusTo(TurretShooterStates.Shooting);
                        intake.setIntakeStatusTo(Intake.IntakeStates.Stop);
                        intake.setShooterStatusTo(Intake.IntakeShooterStates.Shooting);
                        intake.setShooterIsAtTargetRPMStatusTo(shooter.isAtTargetRPM());
                        timer.resetTimer();
                    }
                    else {
                        if (timer.getElapsedTimeSeconds() > 3.5){
                            shooter.setShooterStatusTo(ShooterStates.Stop);
                            turret.setShooterStatusTo(TurretShooterStates.Off);
                            turret.setCurrentPosTo(intake.getEncoderValue());
                            intake.setIntakeStatusTo(Intake.IntakeStates.Intake);
                            setPathState(10);
                        }
                        else{
                            turret.tx = limelight.getTx();
                            turret.setCurrentPosTo(intake.getEncoderValue());
                            intake.setShooterIsAtTargetRPMStatusTo(shooter.isAtTargetRPM());
                            intake.setIntakeStatusTo(Intake.IntakeStates.Stop);
                        }
                    }
                    break;
                }

            case 10: // Path: From Shooting position to prep gather position 3
                turret.setShooterStatusTo(TurretShooterStates.Off);
                turret.setCurrentPosTo(intake.getEncoderValue());
                if(!follower.isBusy()) {
                    shooter.setShooterStatusTo(Shooter.ShooterStates.Stop);
                    intake.setIntakeStatusTo(Intake.IntakeStates.Intake);
                    shooter.periodic();
                    follower.followPath(prepGatherPath3);
                    setPathState(11);
                }
                break;

            case 11: // Path: Gather ball 3
                turret.setShooterStatusTo(TurretShooterStates.Off);
                turret.setCurrentPosTo(intake.getEncoderValue());
                intake.setFirstBallStatusTo(!distSensor.containsFirstBall());
                if(!follower.isBusy()) {
                    setPathState(12);
                    shooter.setShooterStatusTo(Shooter.ShooterStates.Idling);
                    intake.setIntakeStatusTo(Intake.IntakeStates.Intake);
                    shooter.periodic();
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
                        shooter.setShooterStatusTo(ShooterStates.Shooting);
                        turret.tx = limelight.getTx();
                        turret.setCurrentPosTo(intake.getEncoderValue());
                        turret.setShooterStatusTo(TurretShooterStates.Shooting);
                        intake.setIntakeStatusTo(Intake.IntakeStates.Stop);
                        intake.setShooterStatusTo(Intake.IntakeShooterStates.Shooting);
                        intake.setShooterIsAtTargetRPMStatusTo(shooter.isAtTargetRPM());
                        timer.resetTimer();
                    }
                    else {
                        if (timer.getElapsedTimeSeconds() > 3.5){
                            shooter.setShooterStatusTo(ShooterStates.Stop);
                            turret.setShooterStatusTo(TurretShooterStates.Off);
                            turret.setCurrentPosTo(intake.getEncoderValue());
                            intake.setIntakeStatusTo(Intake.IntakeStates.Intake);
                            setPathState(14);
                        }
                        else{
                            turret.tx = limelight.getTx();
                            turret.setCurrentPosTo(intake.getEncoderValue());
                            intake.setShooterIsAtTargetRPMStatusTo(shooter.isAtTargetRPM());
                            intake.setIntakeStatusTo(Intake.IntakeStates.Stop);
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
        intake.periodic();
        shooter.periodic();
        turret.periodic();
        limelight.periodic();
        distSensor.periodic();
        if(shooter.isAtShooterState(ShooterStates.Shooting)) {
            intake.setShooterStatusTo(Intake.IntakeShooterStates.Shooting);
            intake.setShooterIsAtTargetRPMStatusTo(shooter.isAtTargetRPM());
            shooter.updateTargetDistance(0.75);
            turret.tx = limelight.getTx();
            turret.setShooterStatusTo(TurretShooterStates.Shooting);
        }
        else{
            intake.setShooterStatusTo(Intake.IntakeShooterStates.Off);
            turret.setShooterStatusTo(TurretShooterStates.Off);

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
        telemetry.addData("intake state", intake.intakeStatus);
        telemetry.addData("llight dis", limelight.getDis());
        telemetry.addData("Shooter target RPM", shooter.getTargetRPM());
        telemetry.addData("Shooter on target RPM", shooter.isAtTargetRPM());
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
        limelight = new Limelight(hardwareMap);
        limelight.initRedPipeline();
        limelight.startDetect();
        turret = new Turret(hardwareMap);
        distSensor = new DistSensor(hardwareMap);
        intake.initEncoder();
        intake.setIntakeStatusTo(Intake.IntakeStates.Stop);
        shooter.setShooterStatusTo(Shooter.ShooterStates.Stop);
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
