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

    public static final double TO_RPM_CONVERSION_FACTOR = 60.0 / 28.0;
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

    public double getRPM() {
        return ((flywheelMotor1.getVelocity() + flywheelMotor2.getVelocity()) / 2) * TO_RPM_CONVERSION_FACTOR;
    }

    public void setRPM(double targetRPM){
        setBothMotorPower(getCalculatedFlywheelPower(targetRPM));
    }

    public double getCalculatedFlywheelPower(double targetRPM){
        return speedController.calculateFlywheelPower(getRPM(), targetRPM);
    }

    private void setBothMotorPower(double motorPower){
        flywheelMotor1.setPower(motorPower);
        flywheelMotor2.setPower(motorPower);
    }
}
