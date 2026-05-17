package org.firstinspires.ftc.teamcode.subsystems.transfer;


import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
