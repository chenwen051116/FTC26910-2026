package org.firstinspires.ftc.teamcode.shooter;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

@Config
public class SpeedController {
    // Initialize PID Controllers
    private final PIDController flywheelPidController;
    private final PIDController turretPidController;

    // PID Constants for flywheel
    public static double flywheelKp = 0.002;
    public static double flywheelKi = 0;
    public static double flywheelKd = 0.00025;
    public static double flywheelKf = 0;
    public static double flywheelKv = 0.0001955;
    public static double flywheelPidThreshold = 500;
    public static double flywheelPidTolerance = 0.3;

    // PID Constants for turret
    public static double turretKp = 0.0001;
    public static double turretKi = 0.000001;
    public static double turretKd = 0.000005;
    public static double turretKf = 0;
    public static double turretPidTolerance = 0.2;

    // Constructor to initialize all the PID controllers. Must call before using the methods
    // otherwise the PIDControllers will not be initialized.
    public SpeedController() {
        flywheelPidController = new PIDController(flywheelKp, flywheelKi, flywheelKd);
        flywheelPidController.setTolerance(flywheelPidTolerance);

        turretPidController = new PIDController(turretKp, turretKi, turretKd);
        turretPidController.setTolerance(turretPidTolerance);
        turretPidController.setSetPoint(0);

    }

    public double calculateFlywheelPower(double currentRPM, double targetRPM) {
        if (targetRPM > 0){
            double pidCalculationOutput;
            double rpmDifference = targetRPM - currentRPM;
            double pidCalculationInput = rpmDifference / 100;
            double targetMotorPower;

            flywheelPidController.setPID(flywheelKp, flywheelKi, flywheelKd);
            flywheelPidController.setTolerance(flywheelPidTolerance);

            if (Math.abs(rpmDifference) <= flywheelPidThreshold){
                pidCalculationOutput = flywheelPidController.calculate(pidCalculationInput) + flywheelKv * targetRPM;
                targetMotorPower = Math.max(-1.0, Math.min(1.0, pidCalculationOutput));
            } else if (rpmDifference > flywheelPidThreshold){
                // If currentRPM is significantly bigger than the targetRPM, then run at max power
                // to approach ASAP.
                targetMotorPower = 1.0;
            } else {
                // If currentRPM is significantly lower than the targetRPM, then run at 0 power
                // to lower the speed ASAP.
                targetMotorPower = 0.0;
            }
            return targetMotorPower;
        }
        return 0;
    }

    public double calculateTurretPower(double currentPosition, double targetPosition) {
        turretPidController.setSetPoint(targetPosition);
        turretPidController.setPIDF(turretKp, turretKi, turretKd, turretKf);
        return Math.max(-1.0, Math.min(1.0, turretPidController.calculate(currentPosition)));
    }
}
