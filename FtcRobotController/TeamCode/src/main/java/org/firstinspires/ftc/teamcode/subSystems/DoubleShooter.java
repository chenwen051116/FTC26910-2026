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
public class DoubleShooter extends SubsystemBase {

    // shooterLeft and shooterRight to be removed
    public static int baseRPM = 3900;
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
    public static double aimRPM = 4000;
    public static double hoodUpperBar = 1;
    public static double hoodLowerBar = 0.4;
    public static int maxRPM = 4500;
    public static int idleRPM = 3000;
    public static int RPMThreshold = 150;


    // Target RPM for the flywheel
    private double targetRPM = 0.0;

    public double distance = 0;

    public boolean focused = false;
    public boolean autoMode = false;
    public boolean autoLonger = true;


    // The shooter has 3 status: Stop, Idling, and Shooting
    // Stop: The flywheel stops
    // Idling: The flywheel will run at a lower speed, particularly at aimRPM
    // Shooting: The shooter will shoot at a speed that is determined by the limelight
    public enum ShooterStatus {
        Stop,Idling,Shooting
    }



    public ShooterStatus shooterStatus = ShooterStatus.Stop;



    public DoubleShooter(HardwareMap hardwareMap) {
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        hood = hardwareMap.get(Servo.class, "Hood");
        hood.setPosition(hoodAngle);

        // Initialize PID controller
        pidController = new PIDController(Kp, Ki, Kd);

        // Configure shooter
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        return (rightShooter.getVelocity() + leftShooter.getVelocity()) / 2 * 60.0 / 28.0; // 28 ticks per revolution
    }
    public void setTargetRPM(double targetRPM) {
        this.targetRPM = targetRPM;
        pidController.setSetPoint(0);


    }
    public double getTargetRPM() {
        return targetRPM;
    }
    public boolean isAtTargetRPM() {
        return (getTargetRPM() < getFlyWheelRPM() + RPMThreshold && getTargetRPM() > getFlyWheelRPM() - RPMThreshold)&&getFlyWheelRPM()>1000;
    }

    // Store current motor power for telemetry/graphing
    private double currentMotorPower = 0.0;
    private double currentPIDOutput = 0.0;

    /**
     * Update PID controller and set motor powers
     * Call this method in main loop for continuous control
     */
    public void setToShooting(){
        shooterStatus = ShooterStatus.Shooting;
    }
    public void setToStop(){
        shooterStatus = ShooterStatus.Stop;
    }


    public void setToIdle(){
        shooterStatus = ShooterStatus.Idling;
    }
    public void updateFlywheelPID() {
        leftShooter.setVelocity(targetRPM*28/60);
        rightShooter.setVelocity(targetRPM*28/60);
    }

    /**
     * Set flywheel power directly (bypasses PID)
     */
    public void setFlywheelPower(double power) {
        leftShooter.setPower(power);
        rightShooter.setPower(power);
        // Reset target when using manual power
        targetRPM = 0;
    }

    public void completeStop() {
        setFlywheelPower(0);
        pidController.reset();
    }

    // ManualRPM Setting
    public void toggleRPM() {
        setTargetRPM(aimRPM);
        shooterStatus = ShooterStatus.Shooting;
    }

    // Hood
    public void updateHoodAngle(){
        hood.setPosition(hoodAngle);
    }

    // Set the angle of the hood
    public void setHoodAngle(double angle){
        if(angle<hoodLowerBar){
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
//        setTargetRPM(publicRPM);
        distance = abs(distance);
//        if (distance > 3.25){
//            setTargetRPM(3850);
//        }

        // target code

        if (distance < 1.1&&distance>0.58){
            setTargetRPM(1000*distance+baseRPM);
            setHoodAngle(0.2*distance+0.28);
        }
        else{
            setHoodAngle(hoodLowerBar);
            setTargetRPM(maxRPM);
        }


//        else{
//            setTargetRPM(300*distance+2750);
//        }
//
//        if (distance < 0.01){
//            setTargetRPM(3500);
//        }
//
        //        if(automode&&autoLonger){
//            setTargetRPM(3500);
//        }
//        else if(automode&&!autoLonger){
//            setTargetRPM(3100);
//        }
    }

    public void passRPM(){
        targetRPM = aimRPM;
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
        updateFlywheelPID();
        if(shooterStatus == ShooterStatus.Shooting){
            updateAim();
            //passRPM();
        }
        else if(shooterStatus == ShooterStatus.Stop){
            completeStop();
        }
        else if(shooterStatus == ShooterStatus.Idling){
            setTargetRPM(idleRPM);
        }




    }
}
