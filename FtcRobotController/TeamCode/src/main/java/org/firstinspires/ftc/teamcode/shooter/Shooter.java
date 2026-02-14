package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Shooter extends SubsystemBase {
    public enum ShooterState{
        OFF,
        IDLE,
        SHOOTING;
    }
    private ShooterState shooterState;
    private ShooterConfig shooterConfig;
    private final boolean isBlue;
    private final Flywheel flywheel;
    private final Turret turret;
    private final Hood hood;
    public static double idleRPM;

    // Constructor
    public Shooter(boolean isBlue, HardwareMap hardwareMap ) {
        this.isBlue = isBlue;

        // Initialize flywheel class
        DcMotorEx flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "flywheel_1");
        DcMotorEx flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheel_2");
        flywheel = new Flywheel(flywheelMotor1, flywheelMotor2);

        // Initialize turret class
        DcMotorEx turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turret = new Turret(turretMotor);

        // Initialize hood
        Servo hoodServo = hardwareMap.get(Servo.class, "hood");
        hood = new Hood(hoodServo);
    }

    // Get the current shooter state
    public ShooterState getShooterState() {
        return shooterState;
    }

    // Set the current shooter state to he target shooter state
    public void setShooterState(ShooterState targetShooterState ) {
        shooterState = targetShooterState;
    }

    // Getters for debugging
    // Get the current angle of turret in radians
    public double getTurretAngle() {
        return turret.getAngle();
    }

    // Get the current position of hood from 0 to 1
    public double getHoodPosition() {
        return hood.getPosition();
    }

    // Get the current RPM of flywheel
    public double getFlywheelRPM() {
        return flywheel.getRPM();
    }

    // Set the shooter config and set the power of different components
    public void setShooterConfig(ShooterConfig targetShooterConfig) {
        shooterConfig = targetShooterConfig;
    }

    // Calculate shooter config based on current shooter state
    private void calculateShooterConfig(){

    }

    // Calculate the distance based on the current position and the target position by pytag
    public double getDistance(Pose currentPose, Pose targetPose ) {
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        double targetX = targetPose.getX();
        double targetY = targetPose.getY();
        return Math.sqrt(Math.abs(currentX * currentX - targetX * targetX) +
                Math.abs(currentY * currentY + targetY * targetY));
    }

    // Update in every single tick of loop
    @Override
    public void periodic() {
        turret.setAngle(shooterConfig.turretAngle );
        flywheel.setRPM(shooterConfig.flywheelRPM );
        hood.setPosition(shooterConfig.hoodPosition );
    }

}
