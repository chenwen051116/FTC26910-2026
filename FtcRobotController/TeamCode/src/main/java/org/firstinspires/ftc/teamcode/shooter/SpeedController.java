package org.firstinspires.ftc.teamcode.shooter;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

@Config
public class SpeedController {
    // Initialize PID Controller for flywheel
    private final PIDController flywheelPidController;

    // PID Constants for flywheel
    public static double flywheelKp = 0.002;
    public static double flywheelKi = 0;
    public static double flywheelKd = 0.00025;
    public static double flywheelKf = 0;
    public static double flywheelKv = 0.0001955;
    public static double flywheelPidThreshold = 500;
    public static double flywheelPidTolerance = 0.3;

    // Constructor to initialize all the PID controllers. Must call before using the methods
    // otherwise the PIDControllers will not be initialized.
    public SpeedController(){
        flywheelPidController = new PIDController(flywheelKp, flywheelKi, flywheelKd);
        flywheelPidController.setTolerance(flywheelPidTolerance);
    }

    public double calculateFlywheelPower (double currentRPM, double targetRPM ){
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
}
