package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flywheel {
    public static final double TO_RPM_CONVERSION_FACTOR = 60.0 / 28.0;
    private final DcMotorEx flywheelMotor1;
    private final DcMotorEx flywheelMotor2;
    private final SpeedController speedController;

    public Flywheel(DcMotorEx flywheel1, DcMotorEx flywheel2) {
        flywheelMotor1 = flywheel1;
        flywheelMotor2 = flywheel2;

        flywheelMotor1.setDirection(DcMotorEx.Direction.FORWARD);
        flywheelMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        flywheelMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        flywheelMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        speedController = new SpeedController();
    }

    // Get the current motor RPM
    public double getRPM() {
        return ((flywheelMotor1.getVelocity() + flywheelMotor2.getVelocity()) / 2) * TO_RPM_CONVERSION_FACTOR;
    }

    // Let both motor to run at targetRPM using pid controller
    public void setRPM(double targetRPM) {
        setBothMotorPower(getCalculatedFlywheelPower(targetRPM));
    }

    public void stop() {
        setBothMotorPower(0);
    }

    // Get the calculatedFlywheelPower from speedController;
    private double getCalculatedFlywheelPower(double targetRPM) {
        return speedController.calculateFlywheelPower(getRPM(), targetRPM);
    }

    // Set the power of both motor to motorPower
    private void setBothMotorPower(double motorPower) {
        flywheelMotor1.setPower(motorPower);
        flywheelMotor2.setPower(motorPower);
    }
}
