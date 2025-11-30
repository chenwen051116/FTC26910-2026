package org.firstinspires.ftc.teamcode.subsystems;

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
    public static int baseRPM = 3900;
    private final DcMotorEx shooter;
    private final Servo Hood;
    private final PIDController pidController;


    // Tunable PID parameters - can be adjusted via FTC Dashboard
    public static double Kp = 10;  // Proportional gain
    public static double Ki = 0; // Integral gain
    public static double Kd = 0;    // Derivative gain
    public static double pidThreshold = 1000.0; // RPM threshold for PID vs full power control
    public static double tolerance = 0.3; // RPM tolerance for "at target" determination

    public static double hoodAngle = 0.5;

    public static double aimRPM = 4000;

    public static int RPMshift = 3400;

    // Records the default value of the hood. Used in resetHoodAngle()

    // Target RPM for the flywheel
    private double targetRPM = 0.0;
    public static double publicRPM = 0.0;

    public double distance = 0;

    public boolean idelOn = false;

    public boolean focused = false;

    public boolean automode = false;

    public boolean autoLonger = true;

    public double PIDoutput;





    public enum ShooterStatus {
        Stop,Idling,Shooting
    }



    public ShooterStatus shooterStatus = ShooterStatus.Stop;



    public Shooter(HardwareMap hardwareMap) {
        // shooterLeft and shooterRight to be removed
//        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
//        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");


        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        Hood = hardwareMap.get(Servo.class, "Hood");
        Hood.setPosition(hoodAngle);
        // Initialize PID controller
        pidController = new PIDController(Kp, Ki, Kd);

        // Configure motors
        // shooterLeft and shooterRight to be removed
//        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);


 //       shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//        shooterLeft.setDirection(DcMotor.Direction.FORWARD);
//        shooterRight.setDirection(DcMotor.Direction.REVERSE);

        // Configure motor modes - only shooterLeft has encoder
//        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Has encoder
      shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // No encoder

        // Set PID tolerance (adjustable via static parameter)
        pidController.setTolerance(tolerance);
    }

    /**
     * Get current flywheel velocity in rad/s
     * Uses shooterLeft (the motor with encoder) for velocity feedback
     */
    public void updateFocused(boolean focus){
        focused = focus;
    }
    public void setShooterStatus(ShooterStatus status){
        shooterStatus = status;
    }
    public double getFlyWheelVelocity() {
        return shooter.getVelocity() * (2.0 * Math.PI) / 60.0; // Convert RPM to rad/s

    }

    public void updateDis(double dis){
        distance = dis;
    }
    /**
     * Get current flywheel RPM
     * Uses shooterLeft (the motor with encoder) for velocity feedback
     * shooterRight runs in open-loop mode (no encoder)
     */
    public double getFlyWheelRPM() {
        // shooterLeft has encoder, so we use its velocity as representative
        // of the entire flywheel speed (both motors should spin at same speed)
        // getVelocity() returns encoder ticks per second, convert to RPM
        return shooter.getVelocity() * 60.0 / 28.0; // 28 ticks per revolution
    }
    public void setTargetRPM(double targetRPM) {
        this.targetRPM = targetRPM;
        pidController.setSetPoint(0);


    }
    public double getTargetRPM() {
        return targetRPM;
    }
    public boolean isAtTargetRPM() {
        return (getTargetRPM() < getFlyWheelRPM() + 150 && getTargetRPM() > getFlyWheelRPM() - 150)&&getFlyWheelRPM()>1000;
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
        shooter.setVelocity(targetRPM*28/60);
    }

    /**
     * Set flywheel power directly (bypasses PID)
     */
    public void setFlywheelPower(double power) {
        shooter.setPower(power);
        // Reset target when using manual power
        targetRPM = 0;
    }

    public void completeStop() {
        setFlywheelPower(0);
        pidController.reset();
    }

    public void toggleRPM() {
        setTargetRPM(aimRPM);
        shooterStatus = ShooterStatus.Shooting;

    }

    public void updateHoodAngle(){
        Hood.setPosition(hoodAngle);
    }

    public void setHoodAngle(double angle){
        if(angle<0.4){
            angle = 0.4;
        }
        if(angle>1){
            angle = 1;
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

        // kp = 10
        // data
        // GROUP | DIST | AIMRPM | HOOD
        // 1 | 0.60 | 4500 | 0.4
        // 2 | 0.70 | 4500 | 0.45
        // 3 | 0.80 | 4500 | 0.45
        // 4 | 0.90 | 4500 | 0.5
        // 5 | 1.00 | 4750 | 0.5
        // 6 | 1.10 | 5000 | 0.5

        //

        // target code

        if (distance < 1.1&&distance>0.58){
            setTargetRPM(1000*distance+baseRPM);
            setHoodAngle(0.2*distance+0.28);
        }
        else{
            setHoodAngle(0.4);
            setTargetRPM(4500);
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
            setTargetRPM(3000);
        }




    }
}
