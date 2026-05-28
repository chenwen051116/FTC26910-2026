package org.firstinspires.ftc.teamcode.subsystems.shooter;

import static org.firstinspires.ftc.teamcode.Constants.Shooter.C_AX;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.C_AY;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.C_VX;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.C_VY;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.LONG_RANGE_DISTANCE;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.LONG_RANGE_HOOD_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.LONG_RANGE_RPM;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.SHORT_RANGE_DISTANCE;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.SHORT_RANGE_HOOD_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.SHORT_RANGE_RPM;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.TURRET_OFFSET;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.subsystems.Overridable;

@Config
public class Shooter extends Overridable {
    public static class ShooterConfig {
        public double turretAngle;
        public double hoodPosition;
        public double flywheelRPM;

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

    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final Turret turret;
    private final Hood hood;
    private final Flywheel flywheel;
    private ShooterState shooterState;
    private ShooterConfig shooterConfig;
    public static double turretOffsetIncrement = 2;
    public static double kvIncrement = 0.000005;
    public static double turretTrackingDirection = -1;
    public static double turretAngleMultiplier = 0.875;

    // Constructor
    public Shooter(
            Gamepad gamepad1,
            Gamepad gamepad2,
            ServoImplEx turretPrimaryServo,
            ServoImplEx turretSecondaryServo,
            Servo hoodServo,
            DcMotorEx flywheelMotor1,
            DcMotorEx flywheelMotor2
    ) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        turret = new Turret(turretPrimaryServo, turretSecondaryServo);
        hood = new Hood(hoodServo);
        flywheel = new Flywheel(flywheelMotor1, flywheelMotor2);
        shooterState = ShooterState.OFF;
        shooterConfig = new ShooterConfig(0, 0, 0);
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

    public double getTurretTargetAngle(){
        return turret.getTargetAngle();
    }

    public double getTurretPower() {
        return turret.getPosition();
    }

    public void resetTurretOffset() {
        turret.resetOffset();
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

    public double getCurrentFlywheelKv() {
        return PIDControllerFactory.FlywheelPIDController.kv;
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
        ShotProfile shotProfile = calculateShotProfile(distance);
        double relativeTurretAngle = normalizeRadians(displacement.getTheta() - robotPose.getHeading());

        return new ShooterConfig(
                turretTrackingDirection * turretAngleMultiplier * relativeTurretAngle,
                shotProfile.hoodPosition,
                shotProfile.flywheelRPM
        );
    }

    private ShotProfile calculateShotProfile(double distance) {
        double shortestDistance = SHORT_RANGE_DISTANCE[0];
        double longestShortDistance = SHORT_RANGE_DISTANCE[SHORT_RANGE_DISTANCE.length - 1];
        double shortestLongDistance = LONG_RANGE_DISTANCE[0];
        double longestLongDistance = LONG_RANGE_DISTANCE[LONG_RANGE_DISTANCE.length - 1];
        double transitionDistance = (longestShortDistance + shortestLongDistance) / 2.0;

        if (distance <= longestShortDistance) {
            return interpolateShotProfile(
                    clamp(distance, shortestDistance, longestShortDistance),
                    SHORT_RANGE_DISTANCE,
                    SHORT_RANGE_HOOD_POSITION,
                    SHORT_RANGE_RPM
            );
        }

        if (distance < shortestLongDistance) {
            if (distance < transitionDistance) {
                return interpolateShotProfile(
                        longestShortDistance,
                        SHORT_RANGE_DISTANCE,
                        SHORT_RANGE_HOOD_POSITION,
                        SHORT_RANGE_RPM
                );
            }

            return interpolateShotProfile(
                    shortestLongDistance,
                    LONG_RANGE_DISTANCE,
                    LONG_RANGE_HOOD_POSITION,
                    LONG_RANGE_RPM
            );
        }

        return interpolateShotProfile(
                clamp(distance, shortestLongDistance, longestLongDistance),
                LONG_RANGE_DISTANCE,
                LONG_RANGE_HOOD_POSITION,
                LONG_RANGE_RPM
        );
    }

    private ShotProfile interpolateShotProfile(
            double distance,
            double[] distances,
            double[] hoodPositions,
            int[] rpms
    ) {
        int index = 0;
        while (index < distances.length - 2 && distance > distances[index + 1]) {
            index++;
        }

        double startDistance = distances[index];
        double endDistance = distances[index + 1];
        double progress = (distance - startDistance) / (endDistance - startDistance);
        double hoodPosition = hoodPositions[index] + (hoodPositions[index + 1] - hoodPositions[index]) * progress;
        double flywheelRPM = rpms[index] + (rpms[index + 1] - rpms[index]) * progress;

        return new ShotProfile(hoodPosition, flywheelRPM);
    }

    private static class ShotProfile {
        final double hoodPosition;
        final double flywheelRPM;

        ShotProfile(double hoodPosition, double flywheelRPM) {
            this.hoodPosition = hoodPosition;
            this.flywheelRPM = flywheelRPM;
        }
    }

    public Pose getTurretPose(Pose robotPose) {
        // Positive TURRET_OFFSET is behind the robot center, opposite the heading vector.
        return new Pose(
                robotPose.getX() - TURRET_OFFSET * Math.cos(robotPose.getHeading()),
                robotPose.getY() - TURRET_OFFSET * Math.sin(robotPose.getHeading()),
                robotPose.getHeading()
        );
    }

    public Vector getDisplacement(Pose goalPose, Pose robotPose){
        Pose turretPose = getTurretPose(robotPose);
        double yDisplacement = goalPose.getY() - turretPose.getY();
        double xDisplacement = goalPose.getX() - turretPose.getX();
        return new Vector(
                Math.hypot(xDisplacement, yDisplacement),
                Math.atan2(yDisplacement, xDisplacement)
        );
    }

    public Pose getGoalPose(boolean isRed){
        return new Pose(isRed ? 144 - 4 : 4, 144 - 8);
    }
    @Override
    public void runWithoutOverride() {


    }

    @Override
    public void runWhenStartingOverride(){
        this.shooterConfig.flywheelRPM = 3125;
        this.shooterConfig.hoodPosition = 0;
    }

    @Override
    public void alwaysRunning() {
        switch (shooterState) {
            case OFF:
                flywheel.setRPM(0);
                break;
            case IDLE:
                flywheel.setRPM(shooterConfig.flywheelRPM);
                break;
            case SHOOTING:
                flywheel.setRPM(shooterConfig.flywheelRPM);
                break;
        }

        if (gamepad2.aWasPressed()) {
            toggleOverrideDriver();
        }

        if (gamepad1.yWasPressed()) {
            // y button changes the shooter state
            if (getShooterState() == ShooterState.OFF) {
                // Set the shooter state to IDLE when the current shooter state is OFF
                setShooterState(ShooterState.IDLE);
            } else {
                // Set the shooter state to OFF when the current shooter state is IDLE
                setShooterState(ShooterState.OFF);
            }
        }

        if (gamepad1.xWasPressed()) {
            if (getShooterState() == ShooterState.IDLE) {
                setShooterState(ShooterState.SHOOTING);

            } else if (getShooterState() == ShooterState.SHOOTING) {
                setShooterState(ShooterState.IDLE);
            }
        }

        if (gamepad2.leftBumperWasPressed()) {
            turret.addOffset(Math.toRadians(-turretOffsetIncrement));
        }

        if (gamepad2.rightBumperWasPressed()) {
            turret.addOffset(Math.toRadians(turretOffsetIncrement));
        }

        if (gamepad2.dpadUpWasPressed()) {
            PIDControllerFactory.FlywheelPIDController.addFlywheelKv(kvIncrement);
        }

        if (gamepad2.dpadDownWasPressed()) {
            PIDControllerFactory.FlywheelPIDController.addFlywheelKv(-kvIncrement);
        }

        hood.setPosition(shooterConfig.hoodPosition);
        turret.setAngle(shooterConfig.turretAngle);
        turret.periodic();
        flywheel.periodic();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double normalizeRadians(double angleRadians) {
        return Math.atan2(Math.sin(angleRadians), Math.cos(angleRadians));
    }
}
