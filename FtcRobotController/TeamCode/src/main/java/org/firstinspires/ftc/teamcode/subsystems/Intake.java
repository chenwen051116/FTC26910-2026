package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


// TODO: Adapt the system into our robot
public class Intake extends SubsystemBase {

    private final DcMotor intake, transfer;

    public IntakeTransferState intakeCurrentState = IntakeTransferState.Intake_Steady;

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
    }


    public void setIntakePower(double power) {

        intake.setPower(power);

    }

    public void setTransferPower(double power) {
        transfer.setPower(power);
    }

    // Enum which stores all the power needed for each state of the intake motors
    public enum IntakeTransferState {
        Suck_In(1,0),
        Split_Out(-0.7, -1),
        Send_It_Up(1,1),
        Intake_Steady(0,0);
        private final double intakePower;
        private final double transferPower;
        // Set update the transfer state
        IntakeTransferState(double InPower, double TrPower) {
            this.intakePower = InPower;
            this.transferPower = TrPower;
        }
    }

    // This function is not necessary
    // used to update the state of the intake motors when called
    public void setIntakeState(IntakeTransferState intakeTransferState) {
        intakeCurrentState = intakeTransferState;
        if(!shooterAuto || autoForce) {
            intake.setPower(intakeCurrentState.intakePower);
            transfer.setPower(intakeCurrentState.transferPower);
        }
        else{
            if(autoTrans){
                intakeCurrentState = IntakeTransferState.Send_It_Up;
            }
            else{
                intakeCurrentState = IntakeTransferState.Intake_Steady;
            }
        }

    }

    // Standardization of the two functions
    public void updateAutoShoot(boolean auto){
        shooterAuto = auto;
    }

    public void updateAutoTrans(boolean auto){
        autoTrans = auto;
    }

    @Override
    public void periodic() { // FTC 0.001s cycle
        if(!shooterAuto || autoForce) {
            // at shooterAuto or autoForce, the power of the DC motors are set separately
            // thus you will need to make sure that the robot is not in these two states
            intake.setPower(intakeCurrentState.intakePower);
            transfer.setPower(intakeCurrentState.transferPower);
        }
        else{
            if(autoTrans){
                // autoTrans is the state of sending the ball from intake position to shooting
                // position
                intakeCurrentState = IntakeTransferState.Send_It_Up;
            }
            else{
                // if not, then the intake doesn't need to do anything
                intakeCurrentState = IntakeTransferState.Intake_Steady;
            }
            // update the power to the motors
            intake.setPower(intakeCurrentState.intakePower);
            transfer.setPower(intakeCurrentState.transferPower);
        }
    }
}
