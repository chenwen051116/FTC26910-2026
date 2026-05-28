package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.hardware.HardwareCommandCache;

public class Hood {
    private final Servo hoodServo;

    public Hood(Servo hood) {
        hoodServo = hood;
        HardwareCommandCache.setServoPosition(hood, 1);
    }

    // Get the current position of hood from 0 to 1
    public double getPosition() {
        return hoodServo.getPosition();
    }

    // Set the hood to the position from 0 to 1
    public void setPosition(double targetPosition) {
        HardwareCommandCache.setServoPosition(hoodServo, Math.max(0, Math.min(1.0, targetPosition)));
    }
}
