package org.firstinspires.ftc.teamcode.subSystems;

import static java.lang.Math.abs;
import static java.lang.Math.floor;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


// TODO: Adapt the system into our robot
@Config
public class Turret extends SubsystemBase {
    // battery is not yet installed and configured
    // shooter is not yet installed and configured

    private final DcMotor turretMotor;

    public boolean shooterAuto = false;

    public boolean autoForce = false;

    public static double kp = -0.2;
    public static double highkp = -1;
    public static double txBar = 5;

    public static int targetPos = 0;

    public int currentPos = 0;

    public double tx = 0;

    // Constructor for intake motors

    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotor.class, "turret");
        // We do not have distance sensor thus the following object should be removed
        // in future updates
        // The intake does not need to necessarily move at steady
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // The transfer has to be steady for the case where there are already balls in the
        // transfer stage

        //intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // Enum which stores all the power needed for each state of the intake motors
    public void initEncoder(){
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void focusMode(){
        if(abs(tx) < txBar){
            turretMotor.setPower(0.3);
            int dpos = (int) floor(kp*tx);
            targetPos += dpos;
            turretMotor.setTargetPosition(targetPos);
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else{
            turretMotor.setPower(1);
            int dpos = (int) floor(highkp*tx);
            targetPos += dpos;
            turretMotor.setTargetPosition(targetPos);
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void centering(){
        turretMotor.setPower(0.4);
        targetPos = 0;
        turretMotor.setTargetPosition(targetPos);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Standardization of the two functions
    public void updateAutoShoot(boolean auto){
        shooterAuto = auto;
    }


    @Override
    public void periodic() { // FTC 0.001s cycle
        currentPos = turretMotor.getCurrentPosition();
        if(shooterAuto || autoForce) {
            // at shooterAuto or autoForce, the power of the DC motors are set separately
            // thus you will need to make sure that the robot is not in these two states
            focusMode();
        }
        else{
            centering();
        }
    }
}
