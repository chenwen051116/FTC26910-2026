package org.firstinspires.ftc.teamcode.shooter;

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

    public Flywheel(HardwareMap hardwareMap) {
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "flywheel_1");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheel_2");

        flywheelMotor1.setDirection(DcMotorEx.Direction.FORWARD);
        flywheelMotor2.setDirection(DcMotorEx.Direction.FORWARD);

        flywheelMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public double getRpm() {
        return ((flywheelMotor1.getVelocity() + flywheelMotor2.getVelocity()) / 2) * TO_RPM_CONVERSION_FACTOR;
    }

    public void setRPM(){

    }
}
