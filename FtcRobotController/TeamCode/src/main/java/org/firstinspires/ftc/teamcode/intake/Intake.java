package org.firstinspires.ftc.teamcode.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {
    public enum IntakeState{
        INTAKE(0.8),
        STOP(0),
        REVERSE(-0.8);

        private final double power;

        IntakeState(double power) {
            this.power = power;
        }
    }

    private final DcMotor intakeMotor;
    private IntakeState intakeState;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
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

    @Override
    public void periodic() {
        intakeMotor.setPower(intakeState.power);
    }
}
