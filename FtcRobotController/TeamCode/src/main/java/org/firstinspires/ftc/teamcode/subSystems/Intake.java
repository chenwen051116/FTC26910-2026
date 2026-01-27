package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends SubsystemBase {

    private final DcMotor intake;
    private final DcMotor transfer;

    public IntakeStates intakeCurrentState = IntakeStates.Stop;

    // set the 3 status as false in default
    public boolean shooterAuto = false;
    public boolean autoTrans = false;

    public boolean autoForce = false;

    // Constructor for intake motors
    public Intake(HardwareMap hardwareMap) {
        // Initialize hardware
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");

        // Configure Zero power behavior for motor
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Configure direction
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    // Set the power of the intake servo system.
    // Range of input: -1 to 1 (0 as steady);
    public void setIntakePower(double power) {
        intake.setPower(power);
    }

    // Set the power of the transfer servo system.
    // Range of input: 0 to 1 (0.5 as steady)
    public void setTransferPower(double power) {
        transfer.setPower(power);
    }

    // Enum which stores all the power needed for each state of the intake motors
    public enum IntakeStates {
        Ball_In(1,0),
        Ball_Out(-0.7, -1),
        Send_Ball(1,1),
        Stop(0,0);
        private final double frontPower;
        private final double backPower;
        // Set update the transfer state
        IntakeStates(double frontPower, double backPower) {
            this.frontPower = frontPower;
            this.backPower = backPower;
        }
    }

    // This function is not necessary
    // used to update the state of the intake motors when called
    public void setIntakeState(IntakeStates intakeState) {
        intakeCurrentState = intakeState;
        if(!shooterAuto || autoForce) {
            setIntakePower(intakeCurrentState.frontPower);
            setTransferPower(intakeCurrentState.backPower);
        } else{
            if(autoTrans){
                intakeCurrentState = IntakeStates.Send_Ball;
            } else{
                intakeCurrentState = IntakeStates.Stop;
            }
            setIntakePower(intakeCurrentState.frontPower);
            setTransferPower(intakeCurrentState.backPower);
        }

    }

    // Standardization of the two functions
    public void updateAutoShoot(boolean state){
        shooterAuto = state;
    }

    public void updateAutoTrans(boolean state){
        autoTrans = state;
    }

    @Override
    public void periodic() { // FTC 0.001s cycle
        if(!shooterAuto || autoForce) {
            setIntakePower(intakeCurrentState.frontPower);
            setTransferPower(intakeCurrentState.backPower);
        } else{
            if(autoTrans){
                intakeCurrentState = IntakeStates.Send_Ball;
            } else{
                intakeCurrentState = IntakeStates.Stop;
            }
            setIntakePower(intakeCurrentState.frontPower);
            setTransferPower(intakeCurrentState.backPower);
        }
    }
}
