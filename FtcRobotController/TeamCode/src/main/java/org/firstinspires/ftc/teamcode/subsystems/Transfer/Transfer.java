package org.firstinspires.ftc.teamcode.subsystems.Transfer;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Transfer extends SubsystemBase {
    private final Gamepad gamepad;
    private final Intake intake;
    private final Gate gate;
    private final BallSensor ballSensor;
    public Transfer(Gamepad gamepad, DcMotor intakeMotor, Servo gateServo, DistanceSensor[] sensors) {
        if (sensors.length != 3) {
            throw new IllegalArgumentException();
        }

        this.gamepad = gamepad;
        intake = new Intake(intakeMotor);
        gate = new Gate(gateServo);
        ballSensor = new BallSensor(sensors);
    }

    public int getBallCount() {
        return ballSensor.getBallCount();
    }

    public void openGate() {
        gate.open();
    }

    public void closeGate() {
        gate.close();
    }

    public void setIntakeState(Intake.IntakeState intakeState) {
        intake.setIntakeState(intakeState);
    }

    @Override
    public void periodic() {
        if (gamepad.right_trigger > 0.1) {
            setIntakeState(Intake.IntakeState.INTAKE);
        } else if (gamepad.dpad_up) {
            setIntakeState(Intake.IntakeState.REVERSE);
        } else {
            setIntakeState(Intake.IntakeState.STOP);
        }

        if (gamepad.dpadDownWasPressed()) {
            gate.toggle();
        }

        intake.periodic();
        ballSensor.periodic();
    }
}

