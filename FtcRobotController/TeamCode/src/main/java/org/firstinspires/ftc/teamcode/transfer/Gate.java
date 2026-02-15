package org.firstinspires.ftc.teamcode.transfer;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate {
    private final double OPEN_POSITION = 0.6;
    private final double CLOSED_POSITION = 0;
    private Servo gateServo;
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
