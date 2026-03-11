package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous(name = "RedNearAuto")
public class RedNearAuto extends OpMode {
    private final Sequencer sequencer = new Sequencer();
    private Shooter shooter;
    private Drivetrain drivetrain;
    private Transfer transfer;
    private LEDSet ledSet;

    public static double defaultMoveMaxPower = 0.6;

    // START POINT
    public static double startX = 110, startY = 113, startHeading = 0.76;
    private final Pose startPose = new Pose(startX, startY, startHeading);

    // SHOOT POINT
    public static double shootX = 91, shootY = 91, shootHeading = 0.76;
    private final Pose shootPose = new Pose(shootX, shootY, shootHeading);

    // BALL 1 START POINT
    public static double ball1StartX = 84, ball1StartY = 74, ball1StartHeading = 0;
    private final Pose ball1StartPose = new Pose(ball1StartX, ball1StartY, ball1StartHeading);

    // BALL 1 END POINT
    public static double ball1EndX = 110, ball1EndY = 74, ball1EndHeading = 0;
    private final Pose ball1EndPose = new Pose(ball1EndX, ball1EndY, ball1EndHeading);

    // BALL 2 START POINT
    public static double ball2StartX = 84, ball2StartY = 50, ball2StartHeading = 0;
    private final Pose ball2StartPose = new Pose(ball2StartX, ball2StartY, ball2StartHeading);

    // BALL 2 END POINT
    public static double ball2EndX = 110, ball2EndY = 50, ball2EndHeading = 0;
    private final Pose ball2EndPose = new Pose(ball2EndX, ball2EndY, ball2EndHeading);

    // BALL 3 START POINT
    public static double ball3StartX = 84, ball3StartY = 26, ball3StartHeading = 0;
    private final Pose ball3StartPose = new Pose(ball3StartX, ball3StartY, ball3StartHeading);

    // BALL 3 END POINT
    public static double ball3EndX = 110, ball3EndY = 26, ball3EndHeading = 0;
    private final Pose ball3EndPose = new Pose(ball3EndX, ball3EndY, ball3EndHeading);

    // BEFORE GATE POSE
    public static double beforeGateX = 96.5, beforeGateY = 67.5, beforeGateHeading = 0;
    private final Pose beforeGatePose = new Pose(beforeGateX, beforeGateY, beforeGateHeading);

    // OPEN GATE POSE
    public static double openGateX = 126.86, openGateY = 46.23, openGateHeading = 0.1157;
    private final Pose openGatePose = new Pose(openGateX, openGateY, openGateHeading);
    

    private DcMotorEx getMotor(String motorName) {
        return hardwareMap.get(DcMotorEx.class, motorName);
    }

    private Servo getServo(String servoName) {
        return hardwareMap.get(Servo.class, servoName);
    }

    private DistanceSensor getDistanceSensor(String sensorName) {
        return hardwareMap.get(DistanceSensor.class, sensorName);
    }

    @Override
    public void init() {
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

        Servo ballIndicator1 = getServo("ball_indicator_1");
        Servo ballIndicator2 = getServo("ball_indicator_2");
        Servo shooterIndicator = getServo("shooter_indicator");

        ledSet = new LEDSet(ballIndicator1, ballIndicator2, shooterIndicator);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(200);

        // Initialize Sequencer
        // From Start pose to shooting pose
        shoot();

        // From shooting pose to get first ball
        intakeAtPos(ball1StartPose, ball1EndPose);

        // Move from first ball pose to shooting
        shoot();

        // From shooting pose to get the second ball
        intakeAtPos(ball2StartPose, ball2EndPose);

        // Move from second ball pose to shooting
        sequencer.run(() -> drivetrain.goTo(ball2StartPose));
        sequencer.waitUntil(() -> !drivetrain.followerIsBusy());
        shoot();

        // Open gate
        sequencer.run(() -> drivetrain.goTo(beforeGatePose));
        sequencer.waitUntil(() -> !drivetrain.followerIsBusy() && drivetrain.isAtPosition(beforeGatePose));
        sequencer.run(() -> drivetrain.goTo(openGatePose));
        sequencer.waitUntil(() -> !drivetrain.followerIsBusy());

        // Move from gate to third ball pose
        intakeAtPos(ball3StartPose, ball3EndPose);

        // Move from third ball pose to shooting
        shoot();
    }

    private void intakeAtPos(Pose startPose, Pose endPose) {
        intakeAtPos(startPose, endPose, defaultMoveMaxPower);
    }


    private void intakeAtPos(Pose startPose, Pose endPose, double maxPower) {
        sequencer.run(() -> drivetrain.goTo(startPose, maxPower));
        sequencer.waitUntil(() -> !drivetrain.followerIsBusy() && drivetrain.isAtPosition(startPose));
        sequencer.run(() -> transfer.setIntakeState(Intake.IntakeState.INTAKE));
        sequencer.run(() -> drivetrain.goTo(endPose, maxPower));
        sequencer.waitUntil(() -> !drivetrain.followerIsBusy());
        sequencer.run(() -> transfer.setIntakeState(Intake.IntakeState.STOP));
    }

    private void shoot() {
        shoot(defaultMoveMaxPower);
    }

    private void shoot(double maxPower) {
        sequencer.run(() -> shooter.setShooterState(Shooter.ShooterState.IDLE));
        sequencer.run(() -> drivetrain.goTo(shootPose, maxPower));
        sequencer.run(() -> transfer.openGate());
        sequencer.waitUntil(() -> !drivetrain.followerIsBusy());
        // Start Shooting
        sequencer.run(() -> shooter.setShooterState(Shooter.ShooterState.SHOOTING));
        sequencer.waitUntil(() -> shooter.isAtTargetRPM() && transfer.isGateOpen());
        sequencer.run(() -> transfer.setIntakeState(Intake.IntakeState.INTAKE));
        sequencer.wait(800);
        // Finish Shooting
        sequencer.run(() -> transfer.closeGate());
        sequencer.run(() -> shooter.setShooterState(Shooter.ShooterState.IDLE));
        sequencer.run(() -> transfer.setIntakeState(Intake.IntakeState.STOP));
    }

    @Override
    public void start() {
        sequencer.begin();
    }

    @Override
    public void loop() {
        shooter.setShooterConfig(shooter.calculateShooterConfig(drivetrain.getCurrentPose(),
                drivetrain.getCurrentVelocity(),
                drivetrain.getCurrentAcceleration(),
                true));
        shooter.alwaysRunning();
        sequencer.update();
        drivetrain.updateFollower();


        telemetry.addData("Current X", drivetrain.getCurrentPose().getX());
        telemetry.addData("Current Y", drivetrain.getCurrentPose().getY());
        telemetry.addData("Shooter Status", shooter.getShooterState() == Shooter.ShooterState.SHOOTING ? "Shooting" :
                shooter.getShooterState() == Shooter.ShooterState.IDLE ? "Idle" : "Off");
        telemetry.addData("target rpm", shooter.getShooterConfig().flywheelRPM);
        telemetry.addData("current rpm", shooter.getFlywheelRPM());
        telemetry.update();
    }
}
