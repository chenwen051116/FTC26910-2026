package org.firstinspires.ftc.teamcode.subsystems.Transfer;

import com.qualcomm.robotcore.hardware.Servo;

public class Gate {
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