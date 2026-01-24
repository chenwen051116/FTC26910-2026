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
    private final PIDController pidController;


    // Tunable PID parameters - can be adjusted via FTC Dashboard
    public static double Kp = 10;  // Proportional gain
    public static double Ki = 0; // Integral gain
    public static double Kd = 0;    // Derivative gain
    public static double pidThreshold = 1000.0; // RPM threshold for PID vs full power control
    public static double tolerance = 0.3; // RPM tolerance for "at target" determination
    public static double hoodAngle = 0.5;
    public static double hoodUpperBar = 1;
    public static double hoodLowerBar = 0.4;
    public static double hoodAngleCoefficient = 0;
    public static double hoodAngleBase = 0;
    public static int maxRPM = 4500;
    public static int idleRPM = 3000;
    public static int RPMThreshold = 150;
    public static int shooterRPMCoefficient = 1000;
    public static int shooterRPMBase = 3900;


    // Target RPM for the flywheel
    private double targetRPM = 0.0;

    public double distance = 0;

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
        pidController = new PIDController(Kp, Ki, Kd);

        // Configrue both motor
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set PID tolerance (adjustable via static parameter)
        pidController.setTolerance(tolerance);
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
        pidController.setSetPoint(0);


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

    // Update PID controller and set motor powers
    public void setVelocity() {
        leftShooter.setVelocity(targetRPM*28/60);
        rightShooter.setVelocity(targetRPM*28/60);
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
        pidController.reset();
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
        if (distance < 1.1&&distance>0.58){
            setTargetRPM(shooterRPMCoefficient*distance+shooterRPMBase);
            setHoodAngle(hoodAngleBase*distance+hoodAngleCoefficient);
        }
        else{
            setHoodAngle(hoodLowerBar);
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
    @Override
    public void periodic(){

        updateHoodAngle();
        setVelocity();
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
