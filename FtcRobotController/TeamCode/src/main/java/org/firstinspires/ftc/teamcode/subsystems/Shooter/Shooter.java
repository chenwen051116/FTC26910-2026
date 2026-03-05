package org.firstinspires.ftc.teamcode.subsystems.Shooter;

import static org.firstinspires.ftc.teamcode.Constants.Shooter.C_AX;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.C_AY;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.C_VX;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.C_VY;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.LONGEST_SHORT_DISTANCE;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.LONG_RANGE_DISTANCE;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.LONG_RANGE_DISTANCE_INTERVAL;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.LONG_RANGE_HOOD_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.LONG_RANGE_RPM;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.SHORT_RANGE_DISTANCE;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.SHORT_RANGE_DISTANCE_INTERVAL;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.SHORT_RANGE_HOOD_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.SHORT_RANGE_RPM;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Overridable;

@Config
public class Shooter extends Overridable {
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
        SHOOTING,
    }

    private final Gamepad gamepad;
    private final Turret turret;
    private final Hood hood;
    private final Flywheel flywheel;
    private ShooterState shooterState;
    private ShooterConfig shooterConfig;
    public static double IDLE_RPM = 2000;

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

    // Set the current shooter state to the target shooter state
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

    public double getFlywheelMotor1RPM(){
        return flywheel.getRPM1();
    }

    public double getFlywheelMotor2RPM(){
        return flywheel.getRPM2();
    }

    public void initTurretEncoder(){
        turret.initEncoder();
    }


    // Calculate shooter config based on position and velocity
    public ShooterConfig calculateShooterConfig(Pose robotPose, Vector robotVelocity, Vector robotAcceleration, boolean isRed) {
        Pose goalPose = getGoalPose(isRed);

        double vx = robotVelocity.getXComponent();
        double vy = robotVelocity.getYComponent();
        double ax = robotAcceleration.getXComponent();
        double ay = robotAcceleration.getYComponent();

        goalPose = new Pose((goalPose.getX() + vx * C_VX + ax * C_AX), (goalPose.getY() + vy * C_VY + ay * C_AY));

        Vector displacement = getDisplacement(goalPose, robotPose);
        double distance = displacement.getMagnitude();
        int index;
        double targetHoodAngle;
        double targetRPM;

        // Using linear approximation to find the ideal RPM

        if (distance < LONGEST_SHORT_DISTANCE) {
            index = (int)Math.floor((distance - SHORT_RANGE_DISTANCE[0]) / SHORT_RANGE_DISTANCE_INTERVAL);
            index = Math.max(0, Math.min(index, SHORT_RANGE_DISTANCE.length - 2));

            targetHoodAngle = (SHORT_RANGE_HOOD_POSITION[index + 1] - SHORT_RANGE_HOOD_POSITION[index])/
                    (SHORT_RANGE_DISTANCE[index + 1] - SHORT_RANGE_DISTANCE[index]) *
                    (distance - SHORT_RANGE_DISTANCE[index]) +
                    SHORT_RANGE_HOOD_POSITION[index];

            targetRPM = (double)(SHORT_RANGE_RPM[index + 1] - SHORT_RANGE_RPM[index])/
                    (SHORT_RANGE_DISTANCE[index + 1] - SHORT_RANGE_DISTANCE[index]) *
                    (distance - SHORT_RANGE_DISTANCE[index]) +
                    SHORT_RANGE_RPM[index];
        } else {
            targetRPM = 0;
            targetHoodAngle = 0;
        }
//            index = (int)Math.floor((distance - LONG_RANGE_DISTANCE[0]) / LONG_RANGE_DISTANCE_INTERVAL);
//            index = Math.max(0, Math.min(index, LONG_RANGE_DISTANCE.length - 2));
//
//            targetHoodAngle = (LONG_RANGE_HOOD_POSITION[index + 1] - LONG_RANGE_HOOD_POSITION[index])/
//                    (LONG_RANGE_DISTANCE[index + 1] - LONG_RANGE_DISTANCE[index]) *
//                    (distance - LONG_RANGE_DISTANCE[index]) +
//                    LONG_RANGE_HOOD_POSITION[index];
//
//            targetRPM = (LONG_RANGE_RPM[index + 1] - LONG_RANGE_RPM[index])/
//                    (LONG_RANGE_DISTANCE[index + 1] - LONG_RANGE_DISTANCE[index]) *
//                    (distance - LONG_RANGE_DISTANCE[index]) +
//                    LONG_RANGE_RPM[index];



        return new ShooterConfig(
                displacement.getTheta() - robotPose.getHeading(),
                targetHoodAngle,
                targetRPM
        );
    }

    public Vector getDisplacement(Pose goalPose, Pose robotPose){
        return new Vector(goalPose.minus(robotPose));
    }

    public Pose getGoalPose(boolean isRed){
        return new Pose(isRed ? 144 - 6 : 6, 144 - 6);
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
    }

    @Override
    public void runWhenStartingOverride(){
        turret.center();
    }

    @Override
    public void alwaysRunning() {
        switch (shooterState) {
            case OFF:
                flywheel.setRPM(0);
                break;
            case IDLE:
                flywheel.setRPM(IDLE_RPM);
                break;
            case SHOOTING:
                hood.setPosition(shooterConfig.hoodPosition);
                flywheel.setRPM(shooterConfig.flywheelRPM);
                break;
        }
        turret.setAngle(shooterConfig.turretAngle);
        turret.periodic();
        flywheel.periodic();
    }
}
