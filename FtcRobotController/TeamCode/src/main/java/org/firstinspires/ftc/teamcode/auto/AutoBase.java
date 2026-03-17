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
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.LEDSet.LEDSet;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer.Transfer;

@Config
public class AutoBase extends OpMode {

    protected final Sequencer sequencer = new Sequencer();
    protected Shooter shooter;
    protected Transfer transfer;
    protected LEDSet ledSet;
    protected Follower follower;
    protected Drivetrain drivetrain;
    protected static TelemetryManager telemetryM;
    protected boolean isRed = true;

    private DcMotorEx getMotor(String motorName) {
        return hardwareMap.get(DcMotorEx.class, motorName);
    }

    private Servo getServo(String servoName) {
        return hardwareMap.get(Servo.class, servoName);
    }

    private DistanceSensor getDistanceSensor(String sensorName) {
        return hardwareMap.get(DistanceSensor.class, sensorName);
    }


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

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        DcMotor frontLeftMotor = getMotor("front_left");
        DcMotor frontRightMotor = getMotor("front_right");
        DcMotor backLeftMotor = getMotor("back_left");
        DcMotor backRightMotor = getMotor("back_right");

        drivetrain = new Drivetrain(gamepad1, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, follower);


        // Initializing Intake
        DcMotor intakeMotor = getMotor("intake");
        Servo gateServo = getServo("gate");

        DistanceSensor[] sensors = {
                getDistanceSensor("distance_sensor_0"),
                getDistanceSensor("distance_sensor_1"),
                getDistanceSensor("distance_sensor_2")
        };

        transfer = new Transfer(gamepad1, intakeMotor, gateServo, sensors);

        // Initializing Shooter
        DcMotorEx turretMotor = getMotor("turret");
        Servo hoodServo = getServo("hood");
        DcMotorEx flywheelMotor1 = getMotor("flywheel_1");
        DcMotorEx flywheelMotor2 = getMotor("flywheel_2");

        shooter = new Shooter(gamepad1, gamepad2, turretMotor, hoodServo, flywheelMotor1, flywheelMotor2);
        shooter.initTurretEncoder();

        Servo ballIndicator1 = getServo("ball_indicator_1");
        Servo ballIndicator2 = getServo("ball_indicator_2");
        Servo shooterIndicator = getServo("shooter_indicator");

        ledSet = new LEDSet(ballIndicator1, ballIndicator2, shooterIndicator);

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
                               double brakingStrength,
                               double brakingStart) {
        return buildPath(startPose, endPose, brakingStrength, brakingStart, 0.997);
    }

    public PathChain buildPath(Pose startPose,
                               Pose endPose,
                               double brakingStrength,
                               double brakingStart,
                               double tValue) {
        return follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setConstantHeadingInterpolation(endPose.getHeading())
                .setTValueConstraint(tValue)
                .setBrakingStrength(brakingStrength)
                .setBrakingStart(brakingStart)
                .setGlobalDeceleration(brakingStrength)
                .build();
    }

    public PathChain buildPathLinearInterpol(Pose startPose,
                                             Pose endPose) {
        return buildPathLinearInterpol(startPose,
                endPose,
                defaultBrakingStrength,
                defaultBrakingStart
        );
    }

    public PathChain buildPathLinearInterpol(Pose startPose,
                                             Pose endPose,
                                             double brakingStrength,
                                             double brakingStart) {
        return buildPathLinearInterpol(startPose,
                endPose,
                brakingStrength,
                brakingStart,
                0.997
        );
    }

    public PathChain buildPathLinearInterpol(Pose startPose,
                                             Pose endPose,
                                             double brakingStrength,
                                             double brakingStart,
                                             double tValue) {
        return follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .setTValueConstraint(tValue)
                .setBrakingStrength(brakingStrength)
                .setBrakingStart(brakingStart)
                .setGlobalDeceleration(brakingStrength)
                .build();

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
        sequencer.run(() -> follower.followPath(beginPathChain, maxPower, true));
        sequencer.waitUntil(() -> follower.isBusy());
        sequencer.waitUntil(() -> !follower.isBusy());
        sequencer.run(() -> follower.followPath(endPathChain, maxPower, true));
        sequencer.waitUntil(() -> follower.isBusy());
        sequencer.waitUntil(() -> !follower.isBusy());
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
        sequencer.run(() -> follower.followPath(pathChain, maxPower, true));
        sequencer.waitUntil(() -> follower.isBusy());
        sequencer.waitUntil(() -> !follower.isBusy());
        sequencer.wait(duration);
    }

    public void shoot(PathChain pathChain) {
        shoot(pathChain, defaultMoveMaxPower);
    }

    public void shoot(PathChain pathChain, double maxPower) {
        sequencer.run(() -> shooter.setShooterState(Shooter.ShooterState.IDLE));
        sequencer.run(() -> follower.followPath(pathChain, maxPower, true));
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
