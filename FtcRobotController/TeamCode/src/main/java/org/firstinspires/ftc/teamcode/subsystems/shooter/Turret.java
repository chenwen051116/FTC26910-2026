package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.hardware.HardwareCommandCache;

@Config
public class Turret {
    public static double minAngleDegrees = -100;
    public static double maxAngleDegrees = 100;
    public static double centerPosition = 0.5;
    public static double servoTravelDegrees = 300;
    public static double minServoPosition = 0;
    public static double maxServoPosition = 1;
    public static double manualMaxSpeedDegreesPerSecond = 120;
    public static double manualDeadband = 0.05;
    public static double manualDirection = 1;
    public static double pwmMin = 500;
    public static double pwmMax = 2500;

    private final ServoImplEx primaryServo;
    private final ServoImplEx secondaryServo;

    private double defaultOffset = 0.05;
    public static double offset = 0.05;
    private double targetAngle = 0;
    private double commandedAngle = 0;
    private double commandedPosition = centerPosition;
    private long lastManualControlTimeNanos = 0;

    public Turret(ServoImplEx primaryServo, ServoImplEx secondaryServo) {
        this.primaryServo = primaryServo;
        this.secondaryServo = secondaryServo;

        primaryServo.setDirection(Servo.Direction.FORWARD);
        secondaryServo.setDirection(Servo.Direction.FORWARD);
        primaryServo.setPwmRange(new PwmControl.PwmRange(pwmMin, pwmMax));
        secondaryServo.setPwmRange(new PwmControl.PwmRange(pwmMin, pwmMax));
        center();
    }

    public void initEncoder() {
        zero();
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }

    public void resetOffset() {
        this.offset = defaultOffset;
    }

    public double getOffset() {
        return offset;
    }

    public void addOffset(double increment) {
        offset += increment;
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
        this.targetAngle = clampAngle    (normalizeRadians(targetAngle));
    }

    public void manualControl(double stickX) {
        double input = Math.abs(stickX) < manualDeadband ? 0 : stickX;
        long now = System.nanoTime();

        if (lastManualControlTimeNanos == 0) {
            lastManualControlTimeNanos = now;
        }

        double seconds = (now - lastManualControlTimeNanos) / 1_000_000_000.0;
        lastManualControlTimeNanos = now;

        double angleDelta = Math.toRadians(manualMaxSpeedDegreesPerSecond) * manualDirection * input * seconds;
        setAngle(targetAngle + angleDelta);
        periodic();
    }

    public void zero() {
        resetOffset();
        lastManualControlTimeNanos = 0;
        center();
    }

    public void center() {
        setAngle(0);
        periodic();
    }

    public void periodic() {
        commandedAngle = clampAngle(targetAngle + offset);
        commandedPosition = angleToPosition(commandedAngle);
        HardwareCommandCache.setServoPosition(primaryServo, commandedPosition);
        HardwareCommandCache.setServoPosition(secondaryServo, commandedPosition);
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
