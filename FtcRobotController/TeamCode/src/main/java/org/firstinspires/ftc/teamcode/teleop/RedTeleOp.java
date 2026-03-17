package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.Constants.Shooter.LONGEST_SHORT_DISTANCE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
@TeleOp(name = "Red TeleOp")
public class RedTeleOp extends LinearOpMode {
    private DcMotorEx getMotor(String motorName) {
        return hardwareMap.get(DcMotorEx.class, motorName);
    }
    private Servo getServo(String servoName) {
        return hardwareMap.get(Servo.class, servoName);
    }
    private DistanceSensor getDistanceSensor(String sensorName) {
        return hardwareMap.get(DistanceSensor.class, sensorName);
    }

    public void runOpMode() {
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

        Shooter shooter = new Shooter(gamepad1, gamepad2, turretMotor, hoodServo, flywheelMotor1, flywheelMotor2);

        Vector toTargetVector;
        Servo ballIndicator1 = getServo("ball_indicator_1");
        Servo ballIndicator2 = getServo("ball_indicator_2");
        Servo shooterIndicator = getServo("shooter_indicator");

        LEDSet ledSet = new LEDSet(ballIndicator1, ballIndicator2, shooterIndicator);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(200);

        drivetrain.initEncoder();

        boolean isShooting = false;

        Shooter.IDLE_RPM = 2800;

        waitForStart();
        while (opModeIsActive()) {
            toTargetVector = new Vector(shooter.getGoalPose(true).minus(drivetrain.getCurrentPose()));


            ledSet.setBallCount(transfer.getBallCount());
            ledSet.setShooterState(shooter.getShooterState());

            Shooter.ShooterConfig shooterConfig = shooter.calculateShooterConfig(drivetrain.getCurrentPose(),
                    drivetrain.getCurrentVelocity(),
                    drivetrain.getCurrentAcceleration(),
                    true);
            shooter.setShooterConfig(shooterConfig);

            if (shooter.getShooterState() == Shooter.ShooterState.SHOOTING) {
                transfer.overrideDriver();
                if (toTargetVector.getMagnitude() < LONGEST_SHORT_DISTANCE) {
                    if (shooter.isAtTargetRPM() && transfer.isGateOpen()) {
                        isShooting = true;
                    }
                    if (isShooting){
                        transfer.setIntakeState(Intake.IntakeState.TRANSFER);
                    }
                } else {
                    if (shooter.isAtTargetRPM()) {
                        transfer.setIntakeState(Intake.IntakeState.TRANSFER);
                    }
                }
            } else {
                isShooting = false;
                transfer.stopOverrideDriver();
            }



            drivetrain.periodic();
            transfer.periodic();
            shooter.periodic();
            ledSet.periodic();
            shooter.periodic();

            telemetry.addData("!!! Current kv", shooter.getCurrentFlywheelKv() * 100000);
            telemetry.addData("!!! Flywheel RPM", shooter.getFlywheelRPM());
            telemetry.addData("!!! Target RPM", shooter.getShooterConfig().flywheelRPM);
            telemetry.addData("!!! Overriding Shooter", shooter.isOverriding());
            telemetry.addData("Follower X", drivetrain.getCurrentPose().getX());
            telemetry.addData("Follower Y", drivetrain.getCurrentPose().getY());
            telemetry.addData("Follower heading", drivetrain.getCurrentPose().getHeading());
            telemetry.addData("Target turret angle", shooterConfig.turretAngle);
            telemetry.addData("Number of balls", transfer.getBallCount());
            telemetry.addData("Turret angle", shooter.getTurretAngle());
            telemetry.addData("Turret power", shooter.getTurretPower());
            telemetry.addData("Flywheel power", shooter.getFlywheelPower());
            telemetry.addData("Hood Position", shooter.getHoodPosition());
            telemetry.addData("Flywheel RPM1", shooter.getFlywheelMotor1RPM());
            telemetry.addData("Flywheel RPM2", shooter.getFlywheelMotor2RPM());
            telemetry.addData("Distance to RED goal",
                    shooter.getDisplacement(shooter.getGoalPose(true), drivetrain.getCurrentPose()).getMagnitude());
            telemetry.update();
            telemetry.update();
        }
    }
}
