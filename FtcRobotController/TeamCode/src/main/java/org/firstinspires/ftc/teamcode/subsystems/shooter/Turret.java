package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Turret {
    public static double minAngleDegrees = -100;
    public static double maxAngleDegrees = 100;
    public static double centerPosition = 0.5;
    public static double servoTravelDegrees = 300;
    public static double minServoPosition = 0;
    public static double maxServoPosition = 1;

    private final Servo primaryServo;
    private final Servo secondaryServo;

    private double offset = 0;
    private double targetAngle = 0;
    private double commandedAngle = 0;
    private double commandedPosition = centerPosition;

    public Turret(Servo primaryServo, Servo secondaryServo) {
        this.primaryServo = primaryServo;
        this.secondaryServo = secondaryServo;

        primaryServo.setDirection(Servo.Direction.FORWARD);
        secondaryServo.setDirection(Servo.Direction.REVERSE);
        center();
    }

    public void initEncoder() {
        center();
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }

    public double getOffset() {
        return offset;
    }

    public void addOffset(double increment) {
        offset += increment;
    }

    public void resetOffset() {
        offset = 0;
    }

    public double getCurrentAngle() {
        return commandedAngle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getPosition() {
        return commandedPosition;
    }

    public void setAngle(double targetAngle) {
        this.targetAngle = clampAngle(normalizeRadians(targetAngle));
    }

    public void center() {
        setAngle(0);
        periodic();
    }

    public void periodic() {
        commandedAngle = clampAngle(targetAngle + offset);
        commandedPosition = angleToPosition(commandedAngle);
        primaryServo.setPosition(commandedPosition);
        secondaryServo.setPosition(commandedPosition);
    }

    private double angleToPosition(double angleRadians) {
        double travelRadians = Math.toRadians(servoTravelDegrees);
        if (travelRadians <= 0) {
            return centerPosition;
        }

        return clamp(centerPosition + angleRadians / travelRadians, minServoPosition, maxServoPosition);
    }

    private double clampAngle(double angleRadians) {
        return clamp(angleRadians, Math.toRadians(minAngleDegrees), Math.toRadians(maxAngleDegrees));
    }

    private double normalizeRadians(double angleRadians) {
        return Math.atan2(Math.sin(angleRadians), Math.cos(angleRadians));
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
