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
    public static double PIDThreshold = 500; // RPM threshold for PID vs full power control
    public static double tolerance = 0.3; // RPM tolerance for "at target" determination
    public static double hoodAngle = 0;
    public static double hoodMaximumAngle = 1;
    public static double hoodMinimumAngle = 0;
    public static double hoodAngleCoefficient = 0.5667;
    public static double hoodAngleBase = -0.2767;
    public static double hoodAngleThreshold = 0.01;
    public static double configHoodAngle = 0.5;
    public static double burstShootingBeginHoodAngle = 1;
    public static double burstShootingEndHoodAngle = 0;
    public static int maxRPM = 3800;
    public static int idleRPM = 2500;
    public static int RPMThreshold = 100;
    public static int shooterRPMCoefficient = 1021;
    public static int shooterRPMBase = 2973;
    public static int configRPM = 3500;
    public static int burstShootingRPM = 4600;




    // Target RPM for the flywheel
    private double targetRPM = 0.0;

    public double distance = 0;
    public double PIDOutput;
    public double differenceToLastRPM = 0;
    public double lastRPM = 0;


    public boolean focused = false;
    public boolean burstShooting = false;
    public boolean beginBurstShooting = false;

    // The shooter has 3 status: Stop, Idling, and Shooting
    // Stop: The flywheel stops
    // Idling: The flywheel will run at a lower speed (idleRPM)
    // Shooting: The shooter will shoot at a speed that is determined by the limelight
    public enum ShooterStates {
        Stop, Idling, Shooting, BurstShooting
    }

    public ShooterStates shooterStatus = ShooterStates.Stop;

    // set shooter status
    public void setShooterStatusTo(ShooterStates targetShooterStatus){
        shooterStatus = targetShooterStatus;
    }
    public ShooterStates getShooterStatus(){
        return shooterStatus;
    }

    public boolean isAtShooterState(ShooterStates shooterState){
        return shooterStatus == shooterState;
    }

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

    // Update focus status
    public void updateFocused(boolean focus){
        focused = focus;
    }

    public void updateTargetDistance(double dis){
        distance = dis;
    }


    // Get the RPM of the shooter flywheel
    public double getFlyWheelRPM() {
        return (getLeftShooterRPM() + getRightShooterRPM())/2; // 28 ticks per revolution
    }

    public double getLeftShooterRPM() {
        return leftShooter.getVelocity()  * 60.0 / 28.0;
    }

    public double getRightShooterRPM() {
        return rightShooter.getVelocity() * 60.0 / 28.0;
    }

    public void setTargetRPMTo(double targetRPM) {
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
    private double currentMotorPIDOutput = 0.0;

    // Update PID controller and set motor powers
    public void updateShooterPID() {
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
            currentMotorPIDOutput = PIDCalculationOutput;

            setShooterPowerTo(targetMotorPower);
        } else{
            currentMotorPower = 0.0;
            currentMotorPIDOutput = 0.0;
            setShooterPowerTo(0.0);
        }
    }

    // Run burst shooting program
    public void runBurstShooting(){

        if (isAtTargetRPM() && hoodIsAtTargetPosition() && !burstShooting){
            burstShooting = true;
        }

        if (burstShooting){
            leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            setHoodAngleTo(burstShootingEndHoodAngle);
            setShooterPowerTo(1);
        }
    }

    public boolean getBurstShootingStatus(){
        return burstShooting;
    }

    // Set shooter power directly bypassing PID controller
    public void setShooterPowerTo(double power) {
        leftShooter.setPower(power);
        rightShooter.setPower(power);
    }

    // Stop the shooters and set the targetRPM to 0 to stop the PID controller
    public void setCompleteStop() {
        setShooterPowerTo(0);
        targetRPM = 0;
        PIDController.reset();
    }

    // Code part for Hood
    public void updateHoodAngle(){
        hood.setPosition(hoodAngle);
    }

    // Set the angle of the hood
    public void setHoodAngleTo(double angle){
        if(angle < hoodMinimumAngle){
            angle = hoodMinimumAngle;
        }
        if(angle > hoodMaximumAngle){
            angle = hoodMaximumAngle;
        }
        hoodAngle = angle;
    }

    // Update the targetRPM using the value from limelight
    public void updateTargetRPMByDistance() {
        distance = abs(distance);
        if (distance < 1.6&&distance>0.5){
            setTargetRPMTo(shooterRPMCoefficient*distance+shooterRPMBase);
            setHoodAngleTo(hoodAngleCoefficient*distance+hoodAngleBase);
        }
        else{
            setHoodAngleTo(hoodMaximumAngle);
            setTargetRPMTo(maxRPM);
        }

//        setHoodAngleTo(configHoodAngle);
//        setTargetRPMTo(configRPM);
    }

    // Debuggers and getters

    // Return the current motor power
    public double getCurrentMotorPower() {
        return currentMotorPower;
    }

    // Return the current motor PID output
    public double getCurrentMotorPIDOutput() {
        return currentMotorPIDOutput;
    }

    // Return the current hood angle
    public double getCurrentHoodAngle(){
        return hood.getPosition();
    }

    public double getTargetHoodAngle(){
        return hoodAngle;
    }

    public boolean hoodIsAtTargetPosition(){
        return hoodAngle - hoodAngleThreshold <= getCurrentHoodAngle() && getCurrentHoodAngle() <= hoodAngle + hoodAngleThreshold;
    }
    @Override
    public void periodic(){

        updateHoodAngle();
        updateShooterPID();


        if (shooterStatus != ShooterStates.BurstShooting){
            beginBurstShooting = false;
            leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        // Update the flywheel mode accordingly from shooter status
        if (shooterStatus == ShooterStates.Shooting) {
            updateTargetRPMByDistance();
        } else if (shooterStatus == ShooterStates.Stop) {
            setCompleteStop();
        } else if (shooterStatus == ShooterStates.Idling) {
            setTargetRPMTo(idleRPM);
        } else if (shooterStatus == ShooterStates.BurstShooting) {
            if (!beginBurstShooting){
                burstShooting = false;
                beginBurstShooting = true;
                setTargetRPMTo(burstShootingRPM);
                setHoodAngleTo(burstShootingBeginHoodAngle);
            }
            runBurstShooting();
        }
    }
}
