package org.firstinspires.ftc.teamcode.subsystems.transfer;

import com.qualcomm.robotcore.hardware.DcMotor;


public class Intake {
    public enum IntakeState {
        TRANSFER(0.9, 0.9),
        INTAKE(1, 1),
        STOP(0, 0),
        REVERSE(-0.8, -0.8);

        private final double intakePower;
        private final double transferPower;

        IntakeState(double intakePower, double transferPower) {
            this.intakePower = intakePower;
            this.transferPower = transferPower;
        }
    }

    private final DcMotor intakeMotor;
    private final DcMotor transferMotor;
    private IntakeState intakeState;

    public Intake(DcMotor intakeMotor, DcMotor transferMotor) {
        this.intakeMotor = intakeMotor;
        this.transferMotor = transferMotor;

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        transferMotor.setDirection(DcMotor.Direction.REVERSE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeState = IntakeState.STOP;
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }


    public void setIntakeState(IntakeState intakeState) {
        this.intakeState = intakeState;
    }

    public void periodic() {
        intakeMotor.setPower(intakeState.intakePower);
        transferMotor.setPower(intakeState.transferPower);
    }
}
