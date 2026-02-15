package org.firstinspires.ftc.teamcode.old.subSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {

    private final DcMotor intake;
    private final DcMotor transfer;

    public IntakeStates intakeStatus = IntakeStates.Stop;

    // set the 3 status as false in default
    public IntakeShooterStates shooterStatus = IntakeShooterStates.Off;
    public boolean shooterAtTargetRPM = false;
    public boolean firstBall = false;
    public boolean onTarget = false;
    public boolean offTargetShooting = false;

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
    public void setShooterStatusTo(IntakeShooterStates targetStatus) {
        shooterStatus = targetStatus;
    }

    public void setShooterIsAtTargetRPMStatusTo(boolean targetStatus){
        shooterAtTargetRPM = targetStatus;
    }

    public void setFirstBallStatusTo(boolean status){
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
        Intake(1,0),
        Outtake(-0.7, -1),
        Feeding(1,.8),
        Slow_Feeding(1, 0.3),
        Burst_Feeding(1, 0.6),
        Stop(0,0);
        private final double frontPower;
        private final double backPower;
        // Set update the transfer state
        IntakeStates(double frontPower, double backPower) {
            this.frontPower = frontPower;
            this.backPower = backPower;
        }
    }

    public enum IntakeShooterStates {
        Shooting,
        Burst_Shooting,
        Off;
    }

    // This function is not necessary
    // used to update the state of the intake motors when called
    public void setIntakeStatusTo(IntakeStates targetIntakeStatus) {
        switch(shooterStatus){
            case Shooting:
                if(shooterAtTargetRPM){
                    targetIntakeStatus = IntakeStates.Feeding;
                }
                else{
                    targetIntakeStatus = IntakeStates.Stop;
                }
                break;

            case Burst_Shooting:
                if(shooterAtTargetRPM){
                    targetIntakeStatus = IntakeStates.Burst_Feeding;
                }
                else{
                    targetIntakeStatus = IntakeStates.Stop;
                }
                break;

            case Off:
                if (firstBall && targetIntakeStatus == IntakeStates.Intake){
                    targetIntakeStatus = IntakeStates.Slow_Feeding;
                }
                break;
        }
        intakeStatus = targetIntakeStatus;
    }


    @Override
    public void periodic() { // FTC 0.001s cycle
        setIntakePowerTo(intakeStatus.frontPower);
        setTransferPowerTo(intakeStatus.backPower);
    }
}
