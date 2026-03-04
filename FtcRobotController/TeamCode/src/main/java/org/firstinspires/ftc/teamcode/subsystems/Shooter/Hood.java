package org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Hood {
    private final Servo hoodServo;

    public Hood(Servo hood) {
        hoodServo = hood;
        hood.setPosition(0);
    }

    // Get the current position of hood from 0 to 1
    public double getPosition() {
        return hoodServo.getPosition();
    }

    // Set the hood to the position from 0 to 1
    public void setPosition(double targetPosition) {
        hoodServo.setPosition(Math.max(0, Math.min(1.0, targetPosition)));
    }
}