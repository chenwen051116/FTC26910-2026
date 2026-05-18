package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathBuilder;
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
    protected enum HeadingInterpolation {
        CONSTANT,
        LINEAR
    }

    protected final Sequencer sequencer = new Sequencer();
    protected Shooter shooter;
    protected Transfer transfer;
    protected LEDSet ledSet;
    protected Follower follower;
    protected Drivetrain drivetrain;
    protected static TelemetryManager telemetryM;
    protected boolean isRed = true;


    public static double defaultMoveMaxPower = 1, defaultIntakeMoveMaxPower = 1;
    public static double defaultBrakingStart = 1;
    public static double defaultBrakingStrength = 0.75;
    public static int defaultIntakeDuration = 1500;
    public static int defaultShootingDuration = 400;
    public static int defaultTimeBeforeShooting = 0;
    public static int defaultOpenGateTime = 400;

    protected Pose startPose;
    protected Pose shootPose;


    @Override
    public void init() {
        setStartPose();
        setShootPose();
        setIsRed();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Drawing.init();

        RobotHardware robot = new RobotHardware(hardwareMap, gamepad1, gamepad2);
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

    public PathChain buildPath(Pose startPose,
                               Pose endPose) {
        return buildPath(startPose, endPose, defaultBrakingStrength, defaultBrakingStart);
    }

    public PathChain buildPath(Pose startPose,
                               Pose endPose,
                               HeadingInterpolation headingInterpolation) {
        return buildPath(startPose,
                endPose,
                defaultBrakingStrength,
                defaultBrakingStart,
                0.997,
                headingInterpolation
        );
    }

    public PathChain buildPath(Pose startPose,
                               Pose endPose,
                               double brakingStrength,
                               double brakingStart) {
        return buildPath(startPose, endPose, brakingStrength, brakingStart, 0.997);
    }

    public PathChain buildPath(Pose startPose,
                               Pose endPose,
                               double brakingStrength,
                               double brakingStart,
                               double tValue) {
        return buildPath(startPose,
                endPose,
                brakingStrength,
                brakingStart,
                tValue,
                HeadingInterpolation.CONSTANT
        );
    }

    public PathChain buildPath(Pose startPose,
                               Pose endPose,
                               double brakingStrength,
                               double brakingStart,
                               HeadingInterpolation headingInterpolation) {
        return buildPath(startPose,
                endPose,
                brakingStrength,
                brakingStart,
                0.997,
                headingInterpolation
        );
    }

    public PathChain buildPath(Pose startPose,
                               Pose endPose,
                               double tValue,
                               HeadingInterpolation headingInterpolation,
                               boolean applyBraking) {
        return buildPath(startPose,
                endPose,
                defaultBrakingStrength,
                defaultBrakingStart,
                tValue,
                headingInterpolation,
                applyBraking
        );
    }

    public PathChain buildPath(Pose startPose,
                               Pose endPose,
                               double brakingStrength,
                               double brakingStart,
                               double tValue,
                               HeadingInterpolation headingInterpolation) {
        return buildPath(startPose,
                endPose,
                brakingStrength,
                brakingStart,
                tValue,
                headingInterpolation,
                true
        );
    }

    public PathChain buildPath(Pose startPose,
                               Pose endPose,
                               double brakingStrength,
                               double brakingStart,
                               double tValue,
                               HeadingInterpolation headingInterpolation,
                               boolean applyBraking) {
        PathBuilder builder = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose));

        if (headingInterpolation == HeadingInterpolation.LINEAR) {
            builder.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());
        } else {
            builder.setConstantHeadingInterpolation(endPose.getHeading());
        }

        builder.setTValueConstraint(tValue);

        if (applyBraking) {
            builder.setBrakingStrength(brakingStrength)
                    .setBrakingStart(brakingStart)
                    .setGlobalDeceleration(brakingStrength);
        }

        return builder.build();
    }

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
        follower.followPath(pathChain, maxPower, holdEnd);
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

        Shooter.ShooterConfig config = shooter.calculateShooterConfig(getCurrentPose(),
                getCurrentVelocity(),
                getCurrentAcceleration(),
                isRed);
        shooter.setShooterConfig(config);
        shooter.alwaysRunning();
        follower.update();
        drivetrain.setLastPose(follower.getPose());

        sequencer.update();

        Shooter.IDLE_RPM = shooter.calculateShooterConfig(shootPose,
                getCurrentVelocity(),
                getCurrentAcceleration(),
                isRed
                ).flywheelRPM;

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
