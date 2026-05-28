package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.led.LEDSet;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Intake;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Transfer;

@Config
public class AutoBase extends OpMode {
    protected final Sequencer sequencer = new Sequencer();
    protected Shooter shooter;
    protected Transfer transfer;
    protected LEDSet ledSet;
    protected Follower follower;
    protected Drivetrain drivetrain;
    protected RobotHardware robot;
    protected static TelemetryManager telemetryM;
    protected boolean isRed = true;


    public static double defaultMoveMaxPower = 1, defaultIntakeMoveMaxPower = 1;
    private static final double UNRESTRICTED_MAX_POWER = 1.0;
    public static double defaultBrakingStrength = 0.5;
    public static int defaultIntakeDuration = 1100;
    public static int defaultShootingDuration = 400;
    public static int defaultTimeBeforeShooting = 0;
    public static int defaultOpenGateTime = 400;

    protected Pose startPose;
    protected Pose shootPose;


    @Override
    public void init() {
        setIsRed();
        setStartPose();
        setShootPose();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Drawing.init();

        robot = new RobotHardware(hardwareMap, gamepad1, gamepad2);
        follower = robot.follower;
        follower.setStartingPose(startPose);

        drivetrain = robot.drivetrain;
        transfer = robot.transfer;
        shooter = robot.shooter;
        ledSet = robot.ledSet;
        shooter.initTurretEncoder();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(200);
        initializePath();
    }

    public void initializePath() {}

    public void intakeAtPos(PathChain beginPathChain, PathChain endPathChain) {
        intakeAtPos(beginPathChain, endPathChain, defaultIntakeMoveMaxPower);
    }

    public void setStartPose() {
        this.startPose = new Pose(startX, startY, startHeading);
    }

    public void setShootPose() {
        this.shootPose = new Pose(shootX, shootY, shootHeading);
    }

    public void setIsRed() {
        this.isRed = true;
    }

    public void intakeAtPos(PathChain beginPathChain, PathChain endPathChain, double maxPower) {
        sequencer.run(() -> transfer.closeGate());
        sequencer.run(() -> transfer.setIntakeState(Intake.IntakeState.INTAKE));
        queuePath(beginPathChain, maxPower, true);
        queuePath(endPathChain, maxPower, true);
    }

    public void intakeToPos(PathChain pathChain) {
        intakeToPos(pathChain, defaultIntakeDuration);
    }

    public void intakeToPos(PathChain pathChain, int duration) {
        intakeToPos(pathChain, duration, defaultIntakeMoveMaxPower);
    }

    public void intakeToPos(PathChain pathChain, int duration, double maxPower) {
        sequencer.run(() -> transfer.closeGate());
        sequencer.run(() -> transfer.setIntakeState(Intake.IntakeState.INTAKE));
        queuePath(pathChain, maxPower, true);
        sequencer.wait(duration);
    }

    public void shoot(PathChain pathChain) {
        shoot(pathChain, defaultMoveMaxPower);
    }

    public void shoot(PathChain pathChain, double maxPower) {
        sequencer.run(() -> shooter.setShooterState(Shooter.ShooterState.IDLE));
        sequencer.run(() -> followPath(pathChain, maxPower, true));
        sequencer.wait(defaultOpenGateTime);
        sequencer.run(() -> transfer.openGate());
        sequencer.run(() -> transfer.setIntakeState(Intake.IntakeState.STOP));
        sequencer.waitUntil(() -> follower.isBusy());
        sequencer.waitUntil(() -> !follower.isBusy());
        // Start Shooting
        sequencer.wait(defaultTimeBeforeShooting);
        sequencer.run(() -> shooter.setShooterState(Shooter.ShooterState.SHOOTING));
        sequencer.waitUntil(() -> shooter.isAtTargetRPM() && transfer.isGateOpen());
        sequencer.run(() -> transfer.setIntakeState(Intake.IntakeState.TRANSFER));
        sequencer.wait(defaultShootingDuration);
        // Finish Shooting
        sequencer.run(() -> shooter.setShooterState(Shooter.ShooterState.IDLE));
        sequencer.run(() -> transfer.setIntakeState(Intake.IntakeState.STOP));
    }

    public void goTo(PathChain pathChain) {
        goTo(pathChain, defaultMoveMaxPower);
    }

    public void goTo(PathChain pathChain, double maxPower) {
        goTo(pathChain, maxPower, true);
    }

    public void goTo(PathChain pathChain, double maxPower, boolean holdEnd){
        queuePath(pathChain, maxPower, holdEnd);
    }

    private void queuePath(PathChain pathChain, double maxPower, boolean holdEnd) {
        sequencer.run(() -> followPath(pathChain, maxPower, holdEnd));
        sequencer.waitUntil(() -> follower.isBusy());
        sequencer.waitUntil(() -> !follower.isBusy());
    }

    public void draw() {
        Drawing.drawDebug(follower);
    }

    @Override
    public void start() {
        sequencer.begin();
    }

    public void followPath(PathChain pathChain) {
        followPath(pathChain, defaultMoveMaxPower);
    }

    public void followPath(PathChain pathChain, double maxPower) {
        followPath(pathChain, maxPower, true);
    }
    public void followPath(PathChain pathChain, double maxPower, boolean holdEnd) {
        follower.followPath(pathChain, UNRESTRICTED_MAX_POWER, holdEnd);
    }

    public Pose getCurrentPose(){
        return follower.getPose();
    }

    public Vector getCurrentVelocity(){
        return follower.getVelocity();
    }

    public Vector getCurrentAcceleration(){
        return follower.getAcceleration();
    }

    @Override
    public void loop() {
        robot.clearBulkCache();

        Shooter.ShooterConfig config = shooter.calculateShooterConfig(getCurrentPose(),
                getCurrentVelocity(),
                getCurrentAcceleration(),
                isRed);
        shooter.setShooterConfig(config);
        shooter.alwaysRunning();
        follower.update();
        drivetrain.setLastPose(follower.getPose());

        sequencer.update();

        draw();
        transfer.alwaysRunning();

        telemetry.addData("Current X", follower.getPose() .getX());
        telemetry.addData("Current Y", follower.getPose().getY());
        telemetry.addData("Shooter Status", shooter.getShooterState() == Shooter.ShooterState.SHOOTING ? "Shooting" :
                shooter.getShooterState() == Shooter.ShooterState.IDLE ? "Idle" : "Off");
        telemetry.addData("target rpm", shooter.getShooterConfig().flywheelRPM);
        telemetry.addData("current rpm", shooter.getFlywheelRPM());
        telemetry.addData("target turret angle", config.turretAngle);
        telemetry.addData("current turret angle", shooter.getTurretAngle());
        telemetry.update();
    }


}
