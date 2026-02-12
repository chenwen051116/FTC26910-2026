package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Turret {
    private final DcMotorEx turretMotor;
    private final SpeedController speedController;
    public static double encoderConstant; // 1 radian = 1 encoder unit * encoderConstant

    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorEx.Direction.FORWARD);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        speedController = new SpeedController();
    }

    // Initialize encoder
    public void initEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Get the current angle of the turret motor in radians
    public double getAngle() {
        return turretMotor.getCurrentPosition() * encoderConstant;
    }

    // Let the turret motor rotate to the desired angle in radians
    public void setAngle(double targetAngle ) {
        turretMotor.setPower(getCalculatedTurretPower(targetAngle));
    }

    public void center() {
        setAngle(0);
    }

    private double getCalculatedTurretPower(double targetAngle){
        return speedController.calculateTurretPower(getAngle(), targetAngle);
    }
}
