package org.firstinspires.ftc.teamcode.subSystems;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Turret extends SubsystemBase {
    // battery is not yet installed and configured
    // shooter is not yet installed and configured

    private final CRServo turretServo;
    private final PIDFController turretPIDController;
    private final PIDController limelightPIDController;

    public boolean aiming = false;

    public static double turretKp = 0.0001;
    public static double turretKi = 0.000001;
    public static double turretKd = 0.000005;
    public static double turretKf = 0;

    public static double limelightKp = 0.01;
    public static double limelightKi = 0;
    public static double limelightKd = 0;
    public static double limelightKf = 0;

    public static double txThreshold = 0.2;
    public static double PIDTolerance = 0.2;
    public static int currentTargetPos = 0;

    public TurretShooterStates turretShooterStatus = TurretShooterStates.Off;
    public int currentPos = 0;

    public double tx = 0;
    public double power = 0;
    public double currentPower = 0;
    public double limelightFocusPower = 0;

    // Constructor for turret motors

    public Turret(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(CRServo.class, "turret");
        turretServo.setDirection(CRServo.Direction.FORWARD);


        limelightPIDController = new PIDController(limelightKp, limelightKi, limelightKd);
        turretPIDController = new PIDFController(turretKp, turretKi, turretKd, turretKf);
        turretPIDController.setSetPoint(0);
    }

    public void aimByLimelight(){
        limelightPIDController.setPIDF(limelightKp, limelightKi, limelightKd, limelightKf);
        limelightPIDController.setTolerance(PIDTolerance);
        limelightPIDController.setSetPoint(0);

        limelightFocusPower = limelightPIDController.calculate(tx);
        setServoPowerTo(limelightFocusPower);
    }

    public void centering(){
        currentTargetPos = 0;
    }

    public void gotoTargetPosition(){
        turretPIDController.setSetPoint(currentTargetPos);
        turretPIDController.setPIDF(turretKp, turretKi, turretKd, turretKf);
        power = turretPIDController.calculate(currentPos);
        setServoPowerTo(power);
    }

    public void setServoPowerTo(double power){
        if (power > 1){
            power = 1;
        } else if (power < -1){
            power = -1;
        }
        currentPower = -power;
    }

    public void updateServoPower(){
        turretServo.setPower(currentPower);
    }

    // Standardization of the two functions
    public void setCurrentPosTo(int inputValue){
        currentPos = (inputValue);
    }

    public enum TurretShooterStates {
        Shooting(true),
        Off(false);
        private final boolean aiming;
        TurretShooterStates(boolean aiming){
            this.aiming = aiming;
        }
    }

    public void setShooterStatusTo(TurretShooterStates targetTurretShooterStatus){
        turretShooterStatus = targetTurretShooterStatus;
        aiming = turretShooterStatus.aiming;
    }

    @Override
    public void periodic() { // FTC 0.001s cycle
        updateServoPower();
        if (aiming){
            aimByLimelight();
        } else{
            centering();
            gotoTargetPosition();
        }
    }
}
