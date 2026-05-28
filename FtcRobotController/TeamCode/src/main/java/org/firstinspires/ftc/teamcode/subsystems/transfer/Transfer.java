package org.firstinspires.ftc.teamcode.subsystems.transfer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystems.Overridable;

public class Transfer extends Overridable {
    private static final int TRANSFER_STOP_BALL_COUNT = 1;

    private final Gamepad gamepad;
    private final Intake intake;
    private final Gate gate;
    private final BallSensor ballSensor;

    public Transfer(Gamepad gamepad, DcMotor intakeMotor, DcMotor transferMotor, Servo gateServo, DistanceSensor[] sensors) {
        if (sensors.length != 3) {
            throw new IllegalArgumentException();
        }

        this.gamepad = gamepad;
        intake = new Intake(intakeMotor, transferMotor);
        gate = new Gate(gateServo);
        ballSensor = new BallSensor(sensors);

        intake.setIntakeState(Intake.IntakeState.STOP);
    }

    public int getBallCount() {
        return ballSensor.getBallCount();
    }

    public void setIntakeState(Intake.IntakeState intakeState) {
        intake.setIntakeState(limitIntakeState(intakeState));
    }

    public void feedShooter() {
        intake.setIntakeState(Intake.IntakeState.TRANSFER);
    }

    public Intake.IntakeState getIntakeState() {
        return intake.getIntakeState();
    }

    public void openGate() {
        gate.open();
    }

    public void closeGate() {
        gate.close();
    }

    public boolean isGateOpen() {
        return gate.isOpen();
    }

    public void toggleGate() {
        gate.toggle();
    }

    // Stop the intake when starting / stopping overriding
    @Override
    public void runWhenStartingOverride() {
        intake.setIntakeState(Intake.IntakeState.STOP);
        gate.open();
    }

    @Override
    public void runWhenStoppingOverride() {
        intake.setIntakeState(Intake.IntakeState.STOP);
        gate.close();
    }

    @Override
    public void runWithoutOverride() {
        if (gamepad.right_trigger > 0.1) {
            setIntakeState(Intake.IntakeState.INTAKE);
        } else if (gamepad.dpad_up) {
            setIntakeState(Intake.IntakeState.REVERSE);
        } else {
            setIntakeState(Intake.IntakeState.STOP);
        }

        if (gateToggleWasPressed()) {
            gate.toggle();
        }
    }

    @Override
    public void alwaysRunning() {
        ballSensor.periodic();
        if (!isOverriding()) {
            intake.setIntakeState(limitIntakeState(intake.getIntakeState()));
        }
        intake.periodic();
    }

    private Intake.IntakeState limitIntakeState(Intake.IntakeState intakeState) {
        if (!isOverriding()
                && intakeState == Intake.IntakeState.INTAKE
                && getBallCount() >= TRANSFER_STOP_BALL_COUNT) {
            return Intake.IntakeState.INTAKE_WITHOUT_TRANSFER;
        }

        return intakeState;
    }

    private boolean gateToggleWasPressed() {
        return gamepad.dpadDownWasPressed();
    }
}

