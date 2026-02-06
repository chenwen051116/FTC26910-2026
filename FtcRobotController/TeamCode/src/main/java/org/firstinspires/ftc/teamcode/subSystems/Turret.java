package org.firstinspires.ftc.teamcode.subSystems;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


// TODO: Adapt the system into our robot
@Config
public class Turret extends SubsystemBase {
    // battery is not yet installed and configured
    // shooter is not yet installed and configured

    private final CRServo turretServo;
    private final DcMotor encoder;
    private final PIDFController turretPIDController;
    private final PIDController limelightPIDController;

    public boolean shooterAuto = false;

    public boolean autoForce = false;

    public static double turretKp = 1;
    public static double turretKi = 0;
    public static double turretKd = 0;
    public static double turretKf = 0;

    public static double limelightKp = 1;
    public static double limelightKi = 0;
    public static double limelightKd = 0;
    public static double limelightKf = 0;

    public static double txThreshold = 0.2;
    public static double PIDTolerance = 0.2;
    public static int posDifferenceThreshold = 10;

    public static int currentTargetPos = 0;
    public int posDifference = 0;

    public int currentPos = 0;

    public double tx = 0;
    public double power = 0;
    public double limelightFocusPower = 0;

    // Constructor for intake motors

    public Turret(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(CRServo.class, "turret");
        encoder = hardwareMap.get(DcMotor.class, "transfer");
        turretServo.setDirection(CRServo.Direction.FORWARD);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        limelightPIDController = new PIDController(limelightKp, limelightKi, limelightKd);
        turretPIDController = new PIDFController(turretKp, turretKi, turretKd, turretKf);
        turretPIDController.setSetPoint(0);
    }

    public void initEncoder(){
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Enum which stores all the power needed for each state of the intake motors

    public void aimByLimelight(){
        limelightPIDController.setPIDF(limelightKp, limelightKi, limelightKd, limelightKf);
        limelightPIDController.setTolerance(PIDTolerance);
        limelightPIDController.setSetPoint(0);

        limelightFocusPower = limelightPIDController.calculate(tx);
        setServoPower(limelightFocusPower);
    }

    public void centering(){
        currentTargetPos = 0;
    }

    public void gotoTargetPosition(){
        turretPIDController.setSetPoint(currentTargetPos);
        turretPIDController.setPIDF(turretKp, turretKi, turretKd, turretKf);
        power = turretPIDController.calculate(currentPos);
        setServoPower(power);
    }

    public void setServoPower(double power){
        if (power > 1){
            power = 1;
        } else if (power < 0){
            power = -1;
        }
        turretServo.setPower((power + 1) / 2);
    }


    // Standardization of the two functions
    public void updateAutoShoot(boolean auto){
        shooterAuto = auto;
    }

    public void updateCurrentPos(){
        currentPos = encoder.getCurrentPosition();
    }


    @Override
    public void periodic() { // FTC 0.001s cycle
        updateCurrentPos();
        gotoTargetPosition();
        if(shooterAuto || autoForce) {
            aimByLimelight();
        }
        else{
            centering();
        }
    }
}
