package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {

    private final DcMotor intake, transfer;

    public IntakeStates intakeCurrentState = IntakeStates.Stop;

    // set the 3 status as false in default
    public boolean shooterAuto = false;
    public boolean autoTrans = false;

    public boolean autoForce = false;

    // Constructor for intake motors
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");


        // The intake does not need to necessarily move at steady
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // The transfer has to be steady for the case where there are already balls in the
        // transfer stage
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void setIntakePower(double power) {

        intake.setPower(power);

    }

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
            intake.setPower(intakeCurrentState.frontPower);
            transfer.setPower(intakeCurrentState.backPower);
        } else{
            if(autoTrans){
                intakeCurrentState = IntakeStates.Send_Ball;
            } else{
                intakeCurrentState = IntakeStates.Stop;
            }
            intake.setPower(intakeCurrentState.frontPower);
            transfer.setPower(intakeCurrentState.backPower);
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
            // at shooterAuto or autoForce, the power of the DC motors are set separately
            // thus you will need to make sure that the robot is not in these two states
            intake.setPower(intakeCurrentState.frontPower);
            transfer.setPower(intakeCurrentState.backPower);
        }
        else{
            if(autoTrans){
                // autoTrans is the state of sending the ball from intake position to shooting
                // position
                intakeCurrentState = IntakeStates.Send_Ball;
            }
            else{
                // if not, then the intake doesn't need to do anything
                intakeCurrentState = IntakeStates.Stop;
            }
            // update the power to the motors
            intake.setPower(intakeCurrentState.frontPower);
            transfer.setPower(intakeCurrentState.backPower);
        }
    }
}
