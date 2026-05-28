package org.firstinspires.ftc.teamcode.subsystems.transfer;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.HardwareCommandCache;

public class Gate {
    private final double CLOSED_POSITION = 0, OPEN_POSITION = 0.8;
    private final double tolerance = 0.1;
    private final Servo gateServo;
    private boolean isOpen;

    public Gate(Servo gateServo) {
        this.gateServo = gateServo;
        gateServo.setDirection(Servo.Direction.FORWARD);
        HardwareCommandCache.setServoPosition(gateServo, CLOSED_POSITION);
        isOpen = false;
    }

    public boolean isOpen() {
        return OPEN_POSITION - tolerance <= getGatePosition() &&
                getGatePosition() <= OPEN_POSITION + tolerance;
    }

    public double getGatePosition() {
        return gateServo.getPosition();
    }

    public void open() {
        HardwareCommandCache.setServoPosition(gateServo, OPEN_POSITION);
        isOpen = true;
    }

    public void close() {
        HardwareCommandCache.setServoPosition(gateServo, CLOSED_POSITION);
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
