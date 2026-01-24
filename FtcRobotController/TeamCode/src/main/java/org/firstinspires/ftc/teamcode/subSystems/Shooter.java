package org.firstinspires.ftc.teamcode.subSystems;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Shooter extends SubsystemBase {

    // shooterLeft and shooterRight to be removed
    private final DcMotorEx leftShooter;
    private final DcMotorEx rightShooter;
    private final Servo hood;
    private final PIDController PIDController;

    // Tunable PID parameters - can be adjusted via FTC Dashboard
    public static double Kp = 0.0020;  // Proportional gain
    public static double Ki = 0; // Integral gain
    public static double Kd = 0.00025;    // Derivative gain
    public static double Kf = 0;
    public static double Kv = 0.0001955;
    public static double PIDThreshold = 300; // RPM threshold for PID vs full power control
    public static double tolerance = 0.3; // RPM tolerance for "at target" determination
    public static double hoodAngle = 0.5;
    public static double hoodUpperBar = 1;
    public static double hoodLowerBar = 0;
    public static double hoodAngleCoefficient = 1.52    ;
    public static double hoodAngleBase = -0.61;
    public static double configHoodAngle = 0.5;
    public static int maxRPM = 3800;
    public static int idleRPM = 2500;
    public static int RPMThreshold = 150;
    public static int shooterRPMCoefficient = 476;
    public static int shooterRPMBase = 3322;
    public static int configRPM = 3500;




    // Target RPM for the flywheel
    private double targetRPM = 0.0;

    public double distance = 0;
    public double PIDOutput;
    public double differenceToLastRPM = 0;
    public double lastRPM = 0;


    public boolean focused = false;
    public boolean autoMode = false;
    public boolean autoLonger = true;

    // The shooter has 3 status: Stop, Idling, and Shooting
    // Stop: The flywheel stops
    // Idling: The flywheel will run at a lower speed (idleRPM)
    // Shooting: The shooter will shoot at a speed that is determined by the limelight
    public enum ShooterStatus {
        Stop,Idling,Shooting
    }

    public ShooterStatus shooterStatus = ShooterStatus.Stop;

    public Shooter(HardwareMap hardwareMap) {
        // Initialize hardware
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        hood = hardwareMap.get(Servo.class, "hood");
        hood.setPosition(hoodAngle);

        // Initialize PID controller
        PIDController = new PIDController(Kp, Ki, Kd);

        // Configure both motor
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set PID tolerance (adjustable via static parameter)
        PIDController.setTolerance(tolerance);
    }

    public void updateFocused(boolean focus){
        focused = focus;
    }
    public void setShooterStatus(ShooterStatus status){
        shooterStatus = status;
    }

    public void updateDis(double dis){
        distance = dis;
    }

    // Get the RPM of the shooter flywheel
    public double getFlyWheelRPM() {
        return (getLeftWheelRPM() + getRightWheelRPM())/2; // 28 ticks per revolution
    }

    public double getLeftWheelRPM() {
        return leftShooter.getVelocity()  * 60.0 / 28.0;
    }

    public double getRightWheelRPM() {
        return rightShooter.getVelocity() * 60.0 / 28.0;
    }
    public void setTargetRPM(double targetRPM) {
        this.targetRPM = targetRPM;
        PIDController.setSetPoint(0);
    }

    public double getTargetRPM() {
        return targetRPM;
    }
    public boolean isAtTargetRPM() {
        return (getTargetRPM() < getFlyWheelRPM() + RPMThreshold && getTargetRPM() > getFlyWheelRPM() - RPMThreshold) && getFlyWheelRPM()>1000;
    }

    // Store current motor power for telemetry/graphing
    private double currentMotorPower = 0.0;
    private double currentPIDOutput = 0.0;

    // dist | RPM || hood
    // 0.5  | 3600| 0
    // 0.6  | 3600| 0.5
    // 0.75 | 3650| 0.5
    // 0.925| 3700| 0.75
    // 1    | 3800| 1
    // 1.1  | 3900| 1




    // Update PID controller and set motor powers
    public void updateFlywheelPID() {
        differenceToLastRPM = lastRPM - getFlyWheelRPM();
        lastRPM = getFlyWheelRPM();
        if (targetRPM > 0){
            double PIDCalculationOutput = 0;
            double currentRPM = getFlyWheelRPM();
            double RPMDifference = targetRPM - currentRPM;
            double PIDCalculationInput = RPMDifference / 100;
            double targetMotorPower;
            PIDController.setPID(Kp, Ki, Kd);
            PIDController.setTolerance(tolerance);

            if (abs(RPMDifference) <= PIDThreshold){
                PIDCalculationOutput = PIDController.calculate(PIDCalculationInput) + Kv * targetRPM;
                targetMotorPower = Math.max(-1.0, Math.min(1.0, PIDCalculationOutput));
            } else if (RPMDifference > PIDThreshold){
                targetMotorPower = 1.0;
                PIDCalculationOutput = 1.0;
            } else {
                targetMotorPower = 0.0;
                PIDCalculationOutput = 0.0;
            }
            PIDOutput = targetMotorPower;
            currentMotorPower = targetMotorPower;
            currentPIDOutput = PIDCalculationOutput;

            leftShooter.setPower(targetMotorPower);
            rightShooter.setPower(targetMotorPower);
        } else{
            currentMotorPower = 0.0;
            currentPIDOutput = 0.0;
            leftShooter.setPower(0);
            rightShooter.setPower(0);
        }
    }

    // Set Flywheel power directly bypassing PID controller
    public void setFlywheelPower(double power) {
        leftShooter.setPower(power);
        rightShooter.setPower(power);
        // Reset target when using manual power
        targetRPM = 0;
    }

    // Stop the shooters
    public void completeStop() {
        setFlywheelPower(0);
        PIDController.reset();
    }

    // Hood
    public void updateHoodAngle(){
        hood.setPosition(hoodAngle);
    }

    // Set the angle of the hood
    public void setHoodAngle(double angle){
        if(angle < hoodLowerBar){
            angle = hoodLowerBar;
        }
        if(angle > hoodUpperBar){
            angle = hoodUpperBar;
        }
        hoodAngle = angle;
    }

    public double getHoodAngle(){
        return hoodAngle;
    }

    // TODO: Rewrite the updateAim method
    public void updateAim() {
        distance = abs(distance);
        if (distance < 1.1&&distance>0.5){
            setTargetRPM(shooterRPMCoefficient*distance+shooterRPMBase);
            setHoodAngle(hoodAngleBase*distance+hoodAngleCoefficient);
        }
        else{
            setHoodAngle(hoodUpperBar);
            setTargetRPM(maxRPM);
        }
    }


    /**
     * Get current motor power (for graphing/telemetry)
     */
    public double getCurrentMotorPower() {
        return currentMotorPower;
    }

    /**
     * Get current PID output (for graphing/telemetry)
     */
    public double getCurrentPIDOutput() {
        return currentPIDOutput;
    }

    public double getCurrentHoodPosition(){
        return hood.getPosition();
    }

    @Override
    public void periodic(){

        updateHoodAngle();
        updateFlywheelPID();
        if(shooterStatus == ShooterStatus.Shooting){
            updateAim();
        }
        else if(shooterStatus == ShooterStatus.Stop){
            completeStop();
        }
        else if(shooterStatus == ShooterStatus.Idling){
            setTargetRPM(idleRPM);
        }
    }
}
