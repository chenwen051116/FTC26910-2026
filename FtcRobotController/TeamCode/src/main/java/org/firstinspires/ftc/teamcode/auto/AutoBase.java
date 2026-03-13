package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
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
    protected Drivetrain drivetrain;
    protected Transfer transfer;
    protected LEDSet ledSet;
    protected static TelemetryManager telemetryM;

    private DcMotorEx getMotor(String motorName) {
        return hardwareMap.get(DcMotorEx.class, motorName);
    }

    private Servo getServo(String servoName) {
        return hardwareMap.get(Servo.class, servoName);
    }

    private DistanceSensor getDistanceSensor(String sensorName) {
        return hardwareMap.get(DistanceSensor.class, sensorName);
    }


    public static double defaultMoveMaxPower = 0.9, defaultIntakeMoveMaxPower = 0.8;
    public static double defaultIntakeBrakingDistance = 3, defaultShootingBrakingDistance = 2;
    public static double intakeBrakingStrength = 1.45, shootingBrakingStrength = 1.2;
    public static int defaultIntakeDuration = 800;
    public static int defaultTimeBeforeShooting = 500;

    private final Pose startPose = new Pose(startX, startY, startHeading);
    private final Pose shootPose = new Pose(shootX, shootY, shootHeading);


    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Drawing.init();

        // Initializing Drivetrain
        DcMotor frontLeftMotor = getMotor("front_left");
        DcMotor frontRightMotor = getMotor("front_right");
        DcMotor backLeftMotor = getMotor("back_left");
        DcMotor backRightMotor = getMotor("back_right");

        drivetrain = new Drivetrain(gamepad1, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, Constants.createFollower(hardwareMap));
        drivetrain.initEncoder(startPose);
        drivetrain.overrideDriver();

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

        shooter = new Shooter(gamepad1, turretMotor, hoodServo, flywheelMotor1, flywheelMotor2);
        shooter.initTurretEncoder();

        Servo ballIndicator1 = getServo("ball_indicator_1");
        Servo ballIndicator2 = getServo("ball_indicator_2");
        Servo shooterIndicator = getServo("shooter_indicator");

        ledSet = new LEDSet(ballIndicator1, ballIndicator2, shooterIndicator);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(200);
        initializePath();
    }

    public void initializePath() {

    }

    public void intakeAtPos(Pose startPose, Pose endPose) {
        intakeAtPos(startPose, endPose, defaultIntakeMoveMaxPower);
    }


    public void intakeAtPos(Pose startPose, Pose endPose, double maxPower) {
        sequencer.run(() -> drivetrain.goTo(startPose, maxPower, intakeBrakingStrength));
        sequencer.waitUntil(() -> !drivetrain.followerIsBusy());
        sequencer.run(() -> transfer.setIntakeState(Intake.IntakeState.INTAKE));
        sequencer.run(() -> drivetrain.goTo(endPose, maxPower, intakeBrakingStrength));
        sequencer.waitUntil(() -> !drivetrain.followerIsBusy());
        sequencer.run(() -> transfer.setIntakeState(Intake.IntakeState.STOP));
    }

    public void intakeToPos(Pose intakePose) {
        intakeToPos(intakePose, defaultIntakeDuration, defaultIntakeMoveMaxPower);
    }

    public void intakeToPos(Pose intakePose, int duration, double maxPower) {
        sequencer.run(() -> transfer.setIntakeState(Intake.IntakeState.INTAKE));
        sequencer.run(() -> drivetrain.goTo(intakePose, maxPower, intakeBrakingStrength));
        sequencer.waitUntil(() -> !drivetrain.followerIsBusy());
        sequencer.wait(duration);
        sequencer.run(() -> transfer.setIntakeState(Intake.IntakeState.STOP));
    }

    public void shoot() {
        shoot(defaultMoveMaxPower);
    }

    public void shoot(double maxPower) {
        sequencer.run(() -> shooter.setShooterState(Shooter.ShooterState.IDLE));
        sequencer.run(() -> drivetrain.goTo(shootPose, maxPower, shootingBrakingStrength));
        sequencer.run(() -> transfer.openGate());
        sequencer.waitUntil(() -> !drivetrain.followerIsBusy());
        // Start Shooting
        sequencer.wait(defaultTimeBeforeShooting);
        sequencer.run(() -> shooter.setShooterState(Shooter.ShooterState.SHOOTING));
        sequencer.waitUntil(() -> shooter.isAtTargetRPM() && transfer.isGateOpen());
        sequencer.run(() -> transfer.setIntakeState(Intake.IntakeState.INTAKE));
        sequencer.wait(800);
        // Finish Shooting
        sequencer.run(() -> transfer.closeGate());
        sequencer.run(() -> shooter.setShooterState(Shooter.ShooterState.IDLE));
        sequencer.run(() -> transfer.setIntakeState(Intake.IntakeState.STOP));
    }

    public void goTo(Pose targetPose) {
        goTo(targetPose, defaultMoveMaxPower);
    }

    public void goTo(Pose targetPose, double maxPower) {
        sequencer.run(() -> drivetrain.goTo(targetPose, maxPower));
        sequencer.waitUntil(() -> !drivetrain.followerIsBusy());
    }

    @Override
    public void start() {
        sequencer.begin();
    }

    @Override
    public void loop() {
        Shooter.ShooterConfig config = shooter.calculateShooterConfig(drivetrain.getCurrentPose(),
                drivetrain.getCurrentVelocity(),
                drivetrain.getCurrentAcceleration(),
                true);
        shooter.setShooterConfig(config);
        shooter.alwaysRunning();
        sequencer.update();
        drivetrain.updateFollower();
        drivetrain.draw();
        transfer.alwaysRunning();

        telemetry.addData("Current X", drivetrain.getCurrentPose().getX());
        telemetry.addData("Current Y", drivetrain.getCurrentPose().getY());
        telemetry.addData("Shooter Status", shooter.getShooterState() == Shooter.ShooterState.SHOOTING ? "Shooting" :
                shooter.getShooterState() == Shooter.ShooterState.IDLE ? "Idle" : "Off");
        telemetry.addData("target rpm", shooter.getShooterConfig().flywheelRPM);
        telemetry.addData("current rpm", shooter.getFlywheelRPM());
        telemetry.addData("target turret angle", config.turretAngle);
        telemetry.addData("current turret angle", shooter.getTurretAngle());
        telemetry.update();
    }


}
