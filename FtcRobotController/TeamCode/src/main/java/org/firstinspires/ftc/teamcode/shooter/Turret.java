package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

@Config
public class Turret {
    private final DcMotor turretMotor;
    private final SpeedController speedController;
    public static double MAX_RPM = 6000;
    public static double ENCODER_CONSTANT = Constants.MOTOR_TICKS_PER_MINUTE / MAX_RPM; // 1 radian = 1 encoder unit * encoderConstant

    public Turret(DcMotor motor) {
        turretMotor = motor;
        turretMotor.setDirection(DcMotor.Direction.FORWARD);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        speedController = new SpeedController();
    }

    // Initialize encoder
    public void initEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Get the current angle of the turret motor in radians
    public double getAngle() {
        return turretMotor.getCurrentPosition() * ENCODER_CONSTANT;
    }

    // Let the turret motor rotate to the desired angle in radians
    public void setAngle(double targetAngle) {
        turretMotor.setPower(applyRotationalConstraint(speedController.calculateTurretPower(getAngle(), targetAngle)));
    }

    public void center() {
        setAngle(0);
    }

    // Return the actual angle that the turret need to go, avoid rotations over 180 degrees.
    private double applyRotationalConstraint(double targetAngle ) {
        if (targetAngle < -Math.PI){
            targetAngle += 2 * Math.PI;
        } else if (targetAngle > Math.PI){
            targetAngle -= 2 * Math.PI;
        } else {
            return targetAngle;
        }
        return applyRotationalConstraint(targetAngle);
    }
}
