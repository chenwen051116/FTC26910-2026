package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {

    private final DcMotor intake;
    private final DcMotor transfer;

    public IntakeStates intakeStatus = IntakeStates.Stop;

    // set the 3 status as false in default
    public boolean shooting = false;
    public boolean shooterAtTargetRPM = false;
    public boolean firstBall = false;

    // Constructor for intake motors
    public Intake(HardwareMap hardwareMap) {
        // Initialize hardware
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");

        // Configure Zero power behavior for motor
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Configure direction
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void initEncoder(){
        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Set the shooting state according by input
    public void updateShootingStatus(boolean targetStatus) {
        shooting = targetStatus;
    }

    public void updateShooterIsAtTargetRPMStatus(boolean targetStatus){
        shooterAtTargetRPM = targetStatus;
    }

    public void updateFirstBallStatus(boolean status){
        firstBall = status;
    }

    // Set the power of the intake motor
    public void setIntakePowerTo(double power) {
        intake.setPower(power);
    }

    // Set the power of the transfer motor
    public void setTransferPowerTo(double power) {
        transfer.setPower(power);
    }

    public int getEncoderValue(){
        return transfer.getCurrentPosition();
    }

    // Enum which stores all the power needed for each state of the intake motors
    public enum IntakeStates{
        Ball_In(1,0),
        Ball_Out(-0.7, -1),
        Send_Ball(1,1),
        First_Ball_In(1, 0.3),
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
    public void setIntakeStatusTo(IntakeStates targetIntakeStatus) {
        if (shooting){
            if(shooterAtTargetRPM){
                targetIntakeStatus = IntakeStates.Send_Ball;
            } else{
                targetIntakeStatus = IntakeStates.Stop;
            }
        } else{
            if (firstBall){
                if (targetIntakeStatus == IntakeStates.Ball_In){
                    targetIntakeStatus = IntakeStates.First_Ball_In;
                }
            }
        }
        intakeStatus = targetIntakeStatus;
    }


    @Override
    public void periodic() { // FTC 0.001s cycle
        setIntakePowerTo(intakeStatus.frontPower);
        setTransferPowerTo(intakeStatus.backPower);
    }
}
