package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.HardwareCommandCache;

@Config
@TeleOp(name = "Dashboard Motor Power TeleOp", group = "Testing")
public class DashboardMotorPowerTeleOp extends LinearOpMode {
    public static double frontLeftPower = 0;
    public static double frontRightPower = 0;
    public static double backLeftPower = 0;
    public static double backRightPower = 0;
    public static double intakePower = 0;
    public static double transferPower = 0;
    public static double flywheel1Power = 0;
    public static double flywheel2Power = 0;

    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;
    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;
    private DcMotorEx flywheel1Motor;
    private DcMotorEx flywheel2Motor;

    @Override
    public void runOpMode() {
        HardwareCommandCache.resetCommandCache();

        frontLeftMotor = motor("front_left");
        frontRightMotor = motor("front_right");
        backLeftMotor = motor("back_left");
        backRightMotor = motor("back_right");
        intakeMotor = motor("intake");
        transferMotor = motor("transfer");
        flywheel1Motor = motor("flywheel_1");
        flywheel2Motor = motor("flywheel_2");

        waitForStart();

        while (opModeIsActive()) {
            setMotorPowers();
        }

        stopAllMotors();
    }

    private DcMotorEx motor(String name) {
        return hardwareMap.get(DcMotorEx.class, name);
    }

    private void setMotorPowers() {
        HardwareCommandCache.setMotorPower(frontLeftMotor, clampPower(frontLeftPower));
        HardwareCommandCache.setMotorPower(frontRightMotor, clampPower(frontRightPower));
        HardwareCommandCache.setMotorPower(backLeftMotor, clampPower(backLeftPower));
        HardwareCommandCache.setMotorPower(backRightMotor, clampPower(backRightPower));
        HardwareCommandCache.setMotorPower(intakeMotor, clampPower(intakePower));
        HardwareCommandCache.setMotorPower(transferMotor, clampPower(transferPower));
        HardwareCommandCache.setMotorPower(flywheel1Motor, clampPower(flywheel1Power));
        HardwareCommandCache.setMotorPower(flywheel2Motor, clampPower(flywheel2Power));
    }

    private void stopAllMotors() {
        HardwareCommandCache.forceSetMotorPower(frontLeftMotor, 0);
        HardwareCommandCache.forceSetMotorPower(frontRightMotor, 0);
        HardwareCommandCache.forceSetMotorPower(backLeftMotor, 0);
        HardwareCommandCache.forceSetMotorPower(backRightMotor, 0);
        HardwareCommandCache.forceSetMotorPower(intakeMotor, 0);
        HardwareCommandCache.forceSetMotorPower(transferMotor, 0);
        HardwareCommandCache.forceSetMotorPower(flywheel1Motor, 0);
        HardwareCommandCache.forceSetMotorPower(flywheel2Motor, 0);
    }

    private double clampPower(double power) {
        return Math.max(-1, Math.min(1, power));
    }
}
