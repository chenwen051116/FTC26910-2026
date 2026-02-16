package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
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
        SHOOTING,
    }

    private final Gamepad gamepad;
    private final Flywheel flywheel;
    private final Turret turret;
    private final Hood hood;
    private ShooterState shooterState;
    private ShooterConfig shooterConfig;
    private static final double IDLE_RPM = 4500;

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
                hood.setPosition(shooterConfig.hoodPosition);
                flywheel.setRPM(shooterConfig.flywheelRPM);
                break;
        }

        turret.periodic();
        flywheel.periodic();
    }
}

class Turret {
    public static final double MAX_RPM = 500;
    public static final double RADIANS_PER_TICK = 2 * Math.PI * MAX_RPM / Constants.MOTOR_TICKS_PER_MINUTE;

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
        return toRadians(turretMotor.getCurrentPosition());
    }

    // Get the target angle of the turret motor in radians
    public double getTargetAngle() {
        return targetAngle;
    }

    // Let the turret motor rotate to the desired angle in radians
    public void setAngle(double targetAngle) {
        this.targetAngle = (targetAngle % Math.PI * 2 - targetAngle);
    }

    public void center() {
        setAngle(0);
    }

    private double toTicks(double angleInRadians) {
        return angleInRadians * Constants.Shooter.TURRET_GEAR_RATIO / RADIANS_PER_TICK;
    }

    private double toRadians(double ticks) {
        return ticks * RADIANS_PER_TICK / Constants.Shooter.TURRET_GEAR_RATIO;
    }

    public void periodic() {
        turretMotor.setPower(pidController.calculatePower(turretMotor.getCurrentPosition(), toTicks(targetAngle)));
    }
}

class Hood {
    private final Servo hoodServo;
    public Hood(Servo hood) {
        hoodServo = hood;
    }

    public void setAngle(double targetAngle) {
        hoodServo.setPosition((targetAngle - Constants.Shooter.HOOD_BASE_ANGLE) * Constants.Shooter.HOOD_GEAR_RATIO / Constants.SERVO_RANGE);
    }

    // Get the current position of hood from 0 to 1
    public double getPosition() {
        return hoodServo.getPosition();
    }

    // Set the hood to the position from 0 to 1
    public void setPosition(double targetPosition) {
        hoodServo.setPosition(Math.max(0, Math.min(1.0, targetPosition)));
    }
}

class Flywheel {
    public static final double MAX_RPM = 6000;
    public static final double TICKS_PER_REVOLUTION = Constants.MOTOR_TICKS_PER_MINUTE / MAX_RPM;

    private final DcMotorEx flywheelMotor1;
    private final DcMotorEx flywheelMotor2;
    private final PIDControllerFactory.FlywheelPIDController pidController;
    private double targetRPM = 0;

    public Flywheel(DcMotorEx flywheel1, DcMotorEx flywheel2) {
        flywheelMotor1 = flywheel1;
        flywheelMotor2 = flywheel2;

        flywheelMotor1.setDirection(DcMotorEx.Direction.FORWARD);
        flywheelMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        flywheelMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        flywheelMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheelMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        pidController = PIDControllerFactory.createFlywheelPIDController();
    }

    // Get the current motor RPM
    public double getRPM() {
        return ((flywheelMotor1.getVelocity() + flywheelMotor2.getVelocity()) / 2) / TICKS_PER_REVOLUTION * 60;
    }

    // Let both motor to run at targetRPM using pid controller
    public void setRPM(double targetRPM) {
        this.targetRPM = targetRPM;
    }

    // Set the power of both motor to motorPower
    private void setBothMotorPower(double motorPower) {
        flywheelMotor1.setPower(motorPower);
        flywheelMotor2.setPower(motorPower);
    }

    public void periodic() {
        setBothMotorPower(pidController.calculatePower(getRPM(), targetRPM));
    }
}

class PIDControllerFactory {
    static class TurretPIDController extends PIDController {
        public static final double kp = 0.003, ki = 0.00005, kd = 0.0001;
        public static final double kf = 0;
        public static final double tolerance = 0.01;

        private TurretPIDController() {
            super(kp, ki, kd);
            setTolerance(tolerance);
            setSetPoint(0);
        }

        public double calculatePower(double currentPosition, double targetPosition) {
            setSetPoint(targetPosition);
            setPIDF(kp, ki, kd, kf);
            return Math.max(-1, Math.min(1, calculate(currentPosition)));
        }
    }

    static class FlywheelPIDController extends PIDController {
        public static final double kp = 0.002, ki = 0, kd = 0.00025;
        public static final double kv = 0.0001955;
        public static final double threshold = 500, tolerance = 0.3;

        private FlywheelPIDController() {
            super(kp, ki, kd);
            setTolerance(tolerance);
        }

        public double calculatePower(double currentRPM, double targetRPM) {
            if (targetRPM == 0) {
                return 0;
            }

            double rpmDifference = targetRPM - currentRPM;
            if (rpmDifference > threshold) {
                return 1;
            } else if (rpmDifference < -threshold) {
                return 0;
            }
            double pidOutput = calculate(rpmDifference / 100) + kv * targetRPM;
            return Math.max(-1, Math.min(1, calculate(pidOutput)));
        }
    }

    static TurretPIDController createTurretPIDController() {
        return new TurretPIDController();
    }

    static FlywheelPIDController createFlywheelPIDController() {
        return new FlywheelPIDController();
    }
}