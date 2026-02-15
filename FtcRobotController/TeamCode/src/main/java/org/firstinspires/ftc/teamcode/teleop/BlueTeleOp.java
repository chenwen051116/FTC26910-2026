package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.LEDSet;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

@TeleOp(name = "Blue")
class BlueTeleOp extends LinearOpMode {
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

        Shooter shooter = new Shooter(gamepad1, turretMotor, hoodServo, flywheelMotor1, flywheelMotor2);

        Servo ballIndicator1 = getServo("ball_indicator_1");
        Servo ballIndicator2 = getServo("ball_indicator_2");
        Servo shooterIndicator = getServo("shooter_indicator");

        LEDSet ledSet = new LEDSet(ballIndicator1, ballIndicator2, shooterIndicator);

        waitForStart();
        while (opModeIsActive()) {
            ledSet.setBallCount(transfer.getBallCount());
            ledSet.setShooterState(Shooter.ShooterState.IDLE);



            drivetrain.periodic();
            transfer.periodic();
            ledSet.periodic();
            shooter.periodic();

            telemetry.addData("Number of balls", transfer.getBallCount());
        }
    }
}
