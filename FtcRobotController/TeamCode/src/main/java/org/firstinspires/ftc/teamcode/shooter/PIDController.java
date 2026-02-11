package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PIDController {
    // Initialize PID Controller for flywheel
    private final PIDController flyWheelPidController;

    // PID Constants for flywheel
    public static double flyWheelKp = 0.002;
    public static double flyWheelKi = 0;
    public static double flyWheelKd = 0.00025;
    public static double flyWheelKf = 0;
    public static double flyWheelKv = 0.0001955;
    public static double flyWheelPidThreshold = 500;

    // Constructor to initialize all the PID controllers. Must call before using the methods
    // otherwise the PIDControllers will not be initialized.
    public PIDController(){
        flyWheelPidController = new com.arcrobotics.ftclib.controller.PIDController(flyWheelK)
    }

    public double calculateFlywheelPower (double currentRPM, double targetRPM ){

    }


}
