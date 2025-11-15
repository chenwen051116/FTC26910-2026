package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.floor;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


// TODO: Adapt the system into our robot
public class Turret extends SubsystemBase {
    // battery is not yet installed and configured
    // shooter is not yet installed and configured

    private final DcMotor turretMotor;

    public boolean shooterAuto = false;

    public boolean autoForce = false;

    public static double kp = 10;

    public static int targetpos = 0;

    public double tx =0;

    // Constructor for intake motors
    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotor.class, "turret");
        // We do not have distance sensor thus the following object should be removed
        // in future updates
        // The intake does not need to necessarily move at steady
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // The transfer has to be steady for the case where there are already balls in the
        // transfer stage

        //intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // Enum which stores all the power needed for each state of the intake motors
    public void initEncoder(){
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void focusMode(){
        if(tx < 1){
            turretMotor.setPower(0.4);
        }
        else{
            turretMotor.setPower(0.8);
        }
        int dpos = (int) floor(kp*tx);
        targetpos += dpos;
        turretMotor.setTargetPosition(targetpos);
    }

    public void centering(){
        turretMotor.setPower(0.4);
        targetpos = 0;
        turretMotor.setTargetPosition(targetpos);
    }
    // This function is not necessary
    // used to update the state of the intake motors when called


    // Standardization of the two functions
    public void updateAutoShoot(boolean auto){
        shooterAuto = auto;
    }


    @Override
    public void periodic() { // FTC 0.001s cycle
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
