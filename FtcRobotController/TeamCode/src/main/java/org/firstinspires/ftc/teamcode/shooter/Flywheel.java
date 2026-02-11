package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flywheel {
    public enum FlywheelState {
        OFF,
        SHOOTING,
        IDLE,
    }

    private final DcMotorEx flywheelMotor1;
    private final DcMotorEx flywheelMotor2;
    private final SpeedController speedController;

    public Flywheel(HardwareMap hardwareMap) {
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "flywheel_1");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheel_2");

        flywheelMotor1.setDirection(DcMotorEx.Direction.FORWARD);
        flywheelMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        flywheelMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        flywheelMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        speedController = new SpeedController();
    }

    public double getRpm() {
        return ((flywheelMotor1.getVelocity() + flywheelMotor2.getVelocity()) / 2) * 60.0 / 28.0;
    }

    // Directly set the motor power into the value calculated by PID
    public void setRPM(double targetRPM){
        setBothMotorPower(getCalculatedFlywheelPower(targetRPM));
    }

    // Public double method allowing debug usages
    public double getCalculatedFlywheelPower(double targetRPM){
        return speedController.calculateFlywheelPower(getRpm(), targetRPM);
    }

    // Set the power for both motors
    private void setBothMotorPower(double motorPower){
        flywheelMotor1.setPower(motorPower);
        flywheelMotor2.setPower(motorPower);
    }
}
