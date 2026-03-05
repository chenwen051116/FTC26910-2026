package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LEDSet.LEDSet;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer.Transfer;

import com.pedropathing.geometry.Pose;

public class ExampleAuto extends OpMode {
    public static double tuningFlywheelRPM = 4000;
    public static double tuningTurretAngle = 45;
    public static double tuningHoodAngle = 0;
    private final Sequencer sequencer = new Sequencer();

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
        DcMotor frontLeftMotor = getMotor("front_left");
        DcMotor frontRightMotor = getMotor("front_right");
        DcMotor backLeftMotor = getMotor("back_left");
        DcMotor backRightMotor = getMotor("back_right");

        Drivetrain drivetrain = new Drivetrain(gamepad1, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, Constants.createFollower(hardwareMap));

        DcMotor intakeMotor = getMotor("intake");
        Servo gateServo = getServo("gate");
        DistanceSensor[] sensors = {
                getDistanceSensor("distance_sensor_0"),
                getDistanceSensor("distance_sensor_1"),
                getDistanceSensor("distance_sensor_2")
        };

        Transfer transfer = new Transfer(gamepad1, intakeMotor, gateServo, sensors);

        DcMotorEx turretMotor = getMotor("turret");
        Servo hoodServo = getServo("hood");
        DcMotorEx flywheelMotor1 = getMotor("flywheel_1");
        DcMotorEx flywheelMotor2 = getMotor("flywheel_2");

        Shooter shooter = new Shooter(gamepad1, turretMotor, hoodServo, flywheelMotor1, flywheelMotor2);

        Servo ballIndicator1 = getServo("ball_indicator_1");
        Servo ballIndicator2 = getServo("ball_indicator_2");
        Servo shooterIndicator = getServo("shooter_indicator");

        LEDSet ledSet = new LEDSet(ballIndicator1, ballIndicator2, shooterIndicator);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(200);

        sequencer.run(() -> shooter.setShooterConfig(new Shooter.ShooterConfig(Math.toRadians(tuningTurretAngle), Math.toRadians(tuningHoodAngle), tuningFlywheelRPM)));
        sequencer.run(() -> drivetrain.goTo(new Pose(72, 72)));
        sequencer.wait(500);
        sequencer.waitUntil(() -> shooter.getFlywheelRPM() == 4000); // need to add tolerance to make it work
        sequencer.run(() -> transfer.setIntakeState(Intake.IntakeState.INTAKE));
        sequencer.wait(2000);
        sequencer.run(() -> transfer.setIntakeState(Intake.IntakeState.STOP));
    }

    @Override
    public void start() {
        sequencer.begin();
    }

    @Override
    public void loop() {
        sequencer.update();
    }
}
