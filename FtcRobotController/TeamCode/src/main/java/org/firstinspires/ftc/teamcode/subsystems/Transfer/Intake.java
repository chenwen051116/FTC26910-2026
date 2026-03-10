package org.firstinspires.ftc.teamcode.subsystems.Transfer;

import com.qualcomm.robotcore.hardware.DcMotor;


public class Intake {
    public enum IntakeState {
        INTAKE(0.9),
        STOP(0),
        REVERSE(-0.8);

        private final double power;

        IntakeState(double power) {
            this.power = power;
        }
    }

    private final DcMotor intakeMotor;
    private IntakeState intakeState;

    public Intake(DcMotor intakeMotor) {
        this.intakeMotor = intakeMotor;
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeState = IntakeState.STOP;
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }


    public void setIntakeState(IntakeState intakeState) {
        this.intakeState = intakeState;
    }

    public void periodic() {
        intakeMotor.setPower(intakeState.power);
    }
}