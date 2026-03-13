package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.shootHeading;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.shootX;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.shootY;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.startHeading;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.startX;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.RedNear.startY;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LEDSet.LEDSet;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer.Transfer;

@Config
public class AutoBase extends OpMode {
    protected Follower follower;
    protected Shooter shooter;
    protected Transfer transfer;
    protected LEDSet ledSet;
    protected static TelemetryManager telemetryM;
    protected Timer timer;
    protected Timer shooterTimer;

    // Initialize Hardware
    private DcMotorEx getMotor(String motorName) {
        return hardwareMap.get(DcMotorEx.class, motorName);
    }

    private Servo getServo(String servoName) {
        return hardwareMap.get(Servo.class, servoName);
    }

    private DistanceSensor getDistanceSensor(String sensorName) {
        return hardwareMap.get(DistanceSensor.class, sensorName);
    }

    private final Pose startPose = new Pose(startX, startY, startHeading);
    private final Pose shootPose = new Pose(shootX, shootY, shootHeading);
    public static double shootingDuration = 1.5, shootingDelay = 0.75;
    public static double intakeBrakingStrength = 1.6, shootingBrakingStrength = 1.6;
    public static double defaultIntakeBrakingDistance = 1.5, defaultShootingBrakingDistance = 1.5;
    protected boolean shooting = false;
    protected boolean startIntake = false;
    protected boolean waitForDelay = false;
    protected boolean runIntake = false;
    protected int pathState = 1;

    public Pose getCurrentPose(){
        return follower.getPose();
    }

    public Vector getCurrentVelocity(){
        return follower.getVelocity();
    }

    public Vector getCurrentAcceleration(){
        return follower.getAcceleration();
    }

    public void shoot(){
        if (!follower.isBusy()) {
            if (!shooting) {
                shooting = true;
                shooter.setShooterState(Shooter.ShooterState.SHOOTING);
                transfer.openGate();
                shooterTimer.resetTimer();
                waitForDelay = true;
            } else {
                if (!waitForDelay && timer.getElapsedTimeSeconds() > shootingDuration) {
                    shooter.setShooterState(Shooter.ShooterState.IDLE);
                    transfer.setIntakeState(Intake.IntakeState.STOP);
                    shooting = false;
                    transfer.closeGate();
                    this.pathState++;
                } else {
                    if (waitForDelay &&
                            shooterTimer.getElapsedTimeSeconds() > shootingDelay &&
                            shooter.isAtTargetRPM() &&
                            transfer.isGateOpen()) {
                        transfer.setIntakeState(Intake.IntakeState.INTAKE);
                        timer.resetTimer();
                        waitForDelay = false;
                    }
                }
            }
        }
    }

    public void intake(PathChain pathChain) {
        if (!follower.isBusy()) {
            if (!startIntake) {
                transfer.setIntakeState(Intake.IntakeState.INTAKE);
                follower.followPath(pathChain);
                startIntake = true;
            } else {
                transfer.setIntakeState(Intake.IntakeState.STOP);
                startIntake = false;
                this.pathState++;
            }
        }
    }

    public void follow(PathChain pathChain) {
        if (!follower.isBusy()) {
            follower.followPath(pathChain);
            this.pathState++;
        }
    }

    public PathChain buildPath(Pose startPose,
                               Pose endPose,
                               double brakingStrength,
                               double brakingStart) {
        return follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setConstantHeadingInterpolation(endPose.getHeading())
                .setBrakingStart(brakingStart)
                .setBrakingStrength(brakingStrength)
                .setTValueConstraint(0.997)
                .build();
    }

    public PathChain buildIntakePath(Pose startPose, Pose endPose) {
        return buildPath(startPose,
                endPose,
                intakeBrakingStrength,
                defaultIntakeBrakingDistance
        );
    }

    public PathChain buildShootingPath(Pose startPose, Pose endPose) {
        return buildPath(startPose,
                endPose,
                shootingBrakingStrength,
                defaultShootingBrakingDistance
        );
    }

    public void updatePath() {}

    public void buildPath() {}

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Drawing.init();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

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
        timer = new Timer();
        shooterTimer = new Timer();

        buildPath();
    }

    @Override
    public void loop() {
        Shooter.ShooterConfig config = shooter.calculateShooterConfig(getCurrentPose(),
                getCurrentVelocity(),
                getCurrentAcceleration(),
                true);
        shooter.setShooterConfig(config);

        follower.update();
        shooter.alwaysRunning();
        transfer.alwaysRunning();
        updatePath();

        telemetry.addData("Current X", follower.getPose() .getX());
        telemetry.addData("Current Y", follower.getPose().getY());
        telemetry.addData("Shooter Status", shooter.getShooterState() == Shooter.ShooterState.SHOOTING ? "Shooting" :
                shooter.getShooterState() == Shooter.ShooterState.IDLE ? "Idle" : "Off");
        telemetry.addData("target rpm", shooter.getShooterConfig().flywheelRPM);
        telemetry.addData("current rpm", shooter.getFlywheelRPM());
        telemetry.addData("target turret angle", config.turretAngle);
        telemetry.addData("current turret angle", shooter.getTurretAngle());
        telemetry.addData("Path state", this.pathState);
        telemetry.addData("Ready to shoot?", shooter.isAtTargetRPM() && transfer.isGateOpen());
        telemetry.addData("Wait for delay?", waitForDelay);
        telemetry.update();
    }

    @Override
    public void start() {
        shooter.setShooterState(Shooter.ShooterState.IDLE);
    }

}
