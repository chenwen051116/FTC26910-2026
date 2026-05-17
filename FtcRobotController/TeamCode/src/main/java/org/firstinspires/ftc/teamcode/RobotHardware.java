package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.led.LEDSet;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Transfer;

public class RobotHardware {
    public static final String INTAKE_MOTOR_NAME = "intake";
    public static final String TRANSFER_MOTOR_NAME = "transfer";
    public static final String TURRET_PRIMARY_SERVO_NAME = "turret";
    public static final String TURRET_SECONDARY_SERVO_NAME = "turret_2";

    public final Follower follower;
    public final Drivetrain drivetrain;
    public final Transfer transfer;
    public final Shooter shooter;
    public final LEDSet ledSet;

    private final HardwareMap hardwareMap;

    public RobotHardware(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        this.hardwareMap = hardwareMap;

        follower = Constants.createFollower(hardwareMap);
        drivetrain = new Drivetrain(
                gamepad1,
                motor("front_left"),
                motor("front_right"),
                motor("back_left"),
                motor("back_right"),
                follower
        );

        transfer = new Transfer(
                gamepad1,
                motor(INTAKE_MOTOR_NAME),
                motor(TRANSFER_MOTOR_NAME),
                servo("gate"),
                new DistanceSensor[]{
                        distanceSensor("distance_sensor_0"),
                        distanceSensor("distance_sensor_1"),
                        distanceSensor("distance_sensor_2")
                }
        );

        shooter = new Shooter(
                gamepad1,
                gamepad2,
                servo(TURRET_PRIMARY_SERVO_NAME),
                servo(TURRET_SECONDARY_SERVO_NAME),
                servo("hood"),
                motor("flywheel_1"),
                motor("flywheel_2")
        );

        ledSet = new LEDSet(
                servo("ball_indicator_1"),
                servo("ball_indicator_2"),
                servo("shooter_indicator")
        );
    }

    private DcMotorEx motor(String name) {
        return hardwareMap.get(DcMotorEx.class, name);
    }

    private Servo servo(String name) {
        return hardwareMap.get(Servo.class, name);
    }

    private DistanceSensor distanceSensor(String name) {
        return hardwareMap.get(DistanceSensor.class, name);
    }
}
