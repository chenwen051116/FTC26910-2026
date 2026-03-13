package org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.PIDControllerFactory;

public class Flywheel {
    public static final double MOTOR_GEAR_RATIO = 1;
    public static final double TICKS_PER_REVOLUTION = Constants.TICKS_PER_REVOLUTION * MOTOR_GEAR_RATIO;

    private final DcMotorEx flywheelMotor1;
    private final DcMotorEx flywheelMotor2;
    private final PIDControllerFactory.FlywheelPIDController pidController;
    private double targetRPM = 0;

    public Flywheel(DcMotorEx flywheel1, DcMotorEx flywheel2) {
        flywheelMotor1 = flywheel1;
        flywheelMotor2 = flywheel2;

        flywheelMotor1.setDirection(DcMotorEx.Direction.FORWARD);
        flywheelMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        flywheelMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        flywheelMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheelMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        pidController = PIDControllerFactory.createFlywheelPIDController();
    }

    // Get the current motor RPM
    public double getRPM() {
        return ((flywheelMotor1.getVelocity() + flywheelMotor2.getVelocity()) / 2) / TICKS_PER_REVOLUTION * 60;
    }

//    public double getPower() {
//        return pidController.calculatePower(getRPM(), targetRPM);
//    }

    // Let both motor to run at targetRPM using pid controller
    public void setRPM(double targetRPM) {
        this.targetRPM = targetRPM;
    }

    // Set the power of both motor to motorPower
    private void setBothMotorPower(double motorPower) {
        flywheelMotor1.setPower(motorPower);
        flywheelMotor2.setPower(motorPower);
    }

    public void periodic() {
        setBothMotorPower(pidController.calculatePower(getRPM(), targetRPM));
    }
}
