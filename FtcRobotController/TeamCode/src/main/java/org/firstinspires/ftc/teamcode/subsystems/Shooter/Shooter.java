package org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Overridable;

@Config
public class Shooter extends Overridable {
    public static class ShooterConfig {
        public final double turretAngle;
        public final double hoodAngle;
        public final double flywheelRPM;

        public ShooterConfig(double turretAngle, double hoodAngle, double flywheelRPM) {
            this.turretAngle = turretAngle;
            this.hoodAngle = hoodAngle;
            this.flywheelRPM = flywheelRPM;
        }
    }

    public enum ShooterState {
        OFF,
        IDLE,
        SHOOTING,
    }

    private final Gamepad gamepad;
    private final Turret turret;
    private final Hood hood;
    private final Flywheel flywheel;
    private ShooterState shooterState;
    private ShooterConfig shooterConfig;
    public static double IDLE_RPM = 4500;

    // Constructor
    public Shooter(Gamepad gamepad, DcMotorEx turretMotor, Servo hoodServo, DcMotorEx flywheelMotor1, DcMotorEx flywheelMotor2) {
        this.gamepad = gamepad;
        turret = new Turret(turretMotor);
        hood = new Hood(hoodServo);
        flywheel = new Flywheel(flywheelMotor1, flywheelMotor2);
        shooterState = ShooterState.OFF;
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
        return turret.getCurrentAngle();
    }

    public double getTurretPower() {
        return turret.getPower();
    }

    // Get the current position of hood from 0 to 1
    public double getHoodPosition() {
        return hood.getPosition();
    }

    // Get the current RPM of flywheel
    public double getFlywheelRPM() {
        return flywheel.getRPM();
    }

    // Get the current power of flywheel
    public double getFlywheelPower(){
        return flywheel.getPower();
    }

    // Check if the shooter is at target RPM
    public boolean isAtTargetRPM() {
        return flywheel.isAtTargetRPM();
    }


    // Calculate shooter config based on position and velocity
    public static ShooterConfig calculateShooterConfig(Pose robotPose, Vector robotVelocity, boolean isRed) {
        Pose goalPose = new Pose(isRed ? 144 - 6 : 6, 144 - 6);
        Vector displacement = new Vector(goalPose.minus(robotPose)).plus(robotVelocity.times(Constants.Shooter.T1));
        Vector ballVelocityRPM = new Vector(new Pose(Constants.Shooter.X1 * displacement.getMagnitude(), Constants.Shooter.Y1));

        return new ShooterConfig(
                displacement.getTheta() - robotPose.getHeading(),
                Math.PI / 2 - ballVelocityRPM.getTheta(),
                ballVelocityRPM.getMagnitude());
    }

    @Override
    public void runWithoutOverride() {
        if (gamepad.yWasPressed()) {
            // y button changes the shooter state
            if (getShooterState() == ShooterState.OFF) {
                // Set the shooter state to IDLE when the current shooter state is OFF
                setShooterState(ShooterState.IDLE);
            } else {
                // Set the shooter state to OFF when the current shooter state is IDLE
                setShooterState(ShooterState.OFF);
            }
        }

        if (gamepad.xWasPressed()) {
            if (getShooterState() == ShooterState.IDLE) {
                setShooterState(ShooterState.SHOOTING);
            } else if (getShooterState() == ShooterState.SHOOTING) {
                setShooterState(ShooterState.IDLE);
            }
        }

        switch (shooterState) {
            case OFF:
                turret.center();
                flywheel.setRPM(0);
                break;
            case IDLE:
                turret.center();
                flywheel.setRPM(IDLE_RPM);
                break;
            case SHOOTING:
                turret.setAngle(shooterConfig.turretAngle);
                hood.setAngle(shooterConfig.hoodAngle);
                flywheel.setRPM(shooterConfig.flywheelRPM);
                break;
        }
    }

    @Override
    public void alwaysRunning() {
        switch (shooterState) {
            case OFF:
                turret.center();
                flywheel.setRPM(0);
                break;
            case IDLE:
                turret.center();
                flywheel.setRPM(IDLE_RPM);
                break;
            case SHOOTING:
                turret.setAngle(shooterConfig.turretAngle);
                hood.setAngle(shooterConfig.hoodAngle);
                flywheel.setRPM(shooterConfig.flywheelRPM);
                break;
        }

        turret.periodic();
        flywheel.periodic();
    }
}
