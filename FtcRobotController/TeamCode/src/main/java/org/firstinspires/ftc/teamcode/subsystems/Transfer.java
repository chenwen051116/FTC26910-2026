package org.firstinspires.ftc.teamcode.subsystems;

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

    @Override
    public void periodic() {
        if (gamepad.right_trigger > 0.1) {
            intake.setIntakeState(Intake.IntakeState.INTAKE);
        } else if (gamepad.dpad_up) {
            intake.setIntakeState(Intake.IntakeState.REVERSE);
        } else {
            intake.setIntakeState(Intake.IntakeState.STOP);
        }

        if (gamepad.dpadDownWasPressed()) {
            gate.toggle();
        }

        intake.periodic();
        ballSensor.periodic();
    }

    public int getBallCount() {
        return ballSensor.getBallCount();
    }
}

class Intake {
    public enum IntakeState {
        INTAKE(1),
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

class Gate {
    private final double CLOSED_POSITION = 0, OPEN_POSITION = 0.8;
    private final Servo gateServo;
    private boolean isOpen;

    public Gate(Servo gateServo) {
        this.gateServo = gateServo;
        gateServo.setDirection(Servo.Direction.FORWARD);
        gateServo.setPosition(CLOSED_POSITION);
        isOpen = false;
    }

    public boolean isOpen() {
        return isOpen;
    }

    public void open() {
        gateServo.setPosition(OPEN_POSITION);
        isOpen = true;
    }

    public void close() {
        gateServo.setPosition(CLOSED_POSITION);
        isOpen = false;
    }

    public void toggle() {
        if (isOpen) {
            close();
        } else {
            open();
        }
    }
}

class BallSensor {
    private static final double MAX_DISTANCE_WHEN_BALL_PRESENT = 4.0;
    private final DistanceSensor[] sensors;
    private int ballCount = 0;

    public BallSensor(DistanceSensor[] sensors) {
        if (sensors.length != 3) {
            throw new IllegalArgumentException();
        }

        this.sensors = sensors;
    }

    public int getBallCount() {
        return ballCount;
    }

    public boolean hasBallAtPosition(int position) {
        return sensors[position].getDistance(DistanceUnit.CM) < MAX_DISTANCE_WHEN_BALL_PRESENT;
    }

    public void periodic() {
        ballCount = 0;
        for (int i = 0; i < 3; i++) {
            if (hasBallAtPosition(i)) {
                ballCount++;
            } else {
                break;
            }
        }
    }
}
