package org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;

public class Turret {
    public static final double MOTOR_GEAR_RATIO = 1;
    public static final double RADIANS_PER_TICK = 2 * Math.PI / Constants.ENCODER_TICKS_PER_REVOLUTION / MOTOR_GEAR_RATIO;

    private final DcMotor turretMotor;
    private final PIDControllerFactory.TurretPIDController pidController;
    private double targetAngle = 0;

    public Turret(DcMotor motor) {
        turretMotor = motor;
        turretMotor.setDirection(DcMotor.Direction.FORWARD);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pidController = PIDControllerFactory.createTurretPIDController();
    }

    public void initEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Get the current angle of the turret motor in radians
    public double getCurrentAngle() {
        return turretMotor.getCurrentPosition() * RADIANS_PER_TICK / Constants.Shooter.TURRET_GEAR_RATIO;
    }

    // Get the target angle of the turret motor in radians
    public double getTargetAngle() {
        return targetAngle;
    }

    // Let the turret motor rotate to the desired angle in radians
    public void setAngle(double targetAngle) {
        this.targetAngle = Math.max((double) -3 /4 * Math.PI, Math.min((double) 3 /4 * Math.PI, (targetAngle % Math.PI * 2 - targetAngle)));
    }

    public void center() {
        setAngle(0);
    }

    private double toTicks(double angleInRadians) {
        return angleInRadians * Constants.Shooter.TURRET_GEAR_RATIO / RADIANS_PER_TICK;
    }

    public double getPower() {
        return turretMotor.getPower();
    }

    public void periodic() {
        turretMotor.setPower(pidController.calculatePower(turretMotor.getCurrentPosition(), toTicks(targetAngle)));
        // turretMotor.setPower(pidController.calculatePower(getCurrentAngle(), targetAngle));
    }
}
