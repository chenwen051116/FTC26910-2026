package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

@Config
public class Shooter extends SubsystemBase {
    public static class ShooterConfig {
        public final double turretAngle;
        public final double hoodPosition;
        public final double flywheelRPM;

        public ShooterConfig(double turretAngle, double hoodPosition, double flywheelRPM) {
            this.turretAngle = turretAngle;
            this.hoodPosition = hoodPosition;
            this.flywheelRPM = flywheelRPM;
        }
    }

    public enum ShooterState {
        OFF,
        IDLE,
        SHOOTING;
    }
    private ShooterState shooterState;
    private ShooterConfig shooterConfig;
    private final Flywheel flywheel;
    private final Turret turret;
    private final Hood hood;
    public static double IDLE_RPM;

    // Constructor
    public Shooter(DcMotorEx turretMotor, Servo hoodServo, DcMotorEx flywheelMotor1, DcMotorEx flywheelMotor2) {
        turret = new Turret(turretMotor);
        hood = new Hood(hoodServo);
        flywheel = new Flywheel(flywheelMotor1, flywheelMotor2);
    }

    // Get the current shooter state
    public ShooterState getShooterState() {
        return shooterState;
    }

    // Set the current shooter state to he target shooter state
    public void setShooterState(ShooterState shooterState) {
        this.shooterState = shooterState;
    }

    public ShooterConfig getShooterConfig() {
        return shooterConfig;
    }

    // Set the shooter config and set the power of different components
    public void setShooterConfig(ShooterConfig targetShooterConfig) {
        shooterConfig = targetShooterConfig;
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

    // Calculate shooter config based on position and velocity
    private static ShooterConfig calculateShooterConfig(Pose robotPose, Vector robotVelocity, boolean isRed) {
        Pose goalPose = new Pose(isRed ? 144 - 6 : 6, 144 - 6);
        Vector displacement = new Vector(goalPose.minus(robotPose)).plus(robotVelocity.times(Constants.Shooter.T1));
        Vector ballVelocityRPM = new Vector(new Pose(Constants.Shooter.X1 * displacement.getMagnitude(), Constants.Shooter.Y1));

        return new ShooterConfig(
                displacement.getTheta() - robotPose.getHeading(),
                Math.PI / 2 - ballVelocityRPM.getTheta(),
                ballVelocityRPM.getMagnitude());
    }

    // Update in every single tick of loop
    @Override
    public void periodic() {
        switch (shooterState) {
            case OFF:
                turret.center();
                flywheel.stop();
                break;
            case IDLE:
                turret.center();
                flywheel.setRPM(IDLE_RPM);
                break;
            case SHOOTING:
                turret.setAngle(shooterConfig.turretAngle);
                flywheel.setRPM(shooterConfig.flywheelRPM);
                hood.setPosition(shooterConfig.hoodPosition);
                break;
        }
    }
}
