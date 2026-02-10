package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

public class BallSensor {
    private final double MAX_DISTANCE_WHEN_BALL_PRESENT = 4.0;
    private final ArrayList<DistanceSensor> sensors = new ArrayList<DistanceSensor>();
    private int ballCount = 0;

    public BallSensor(HardwareMap hardwareMap) {
        sensors.add(hardwareMap.get(DistanceSensor.class, "distance_sensor_0"));
        sensors.add(hardwareMap.get(DistanceSensor.class, "distance_sensor_1"));
        sensors.add(hardwareMap.get(DistanceSensor.class, "distance_sensor_2"));
    }

    public int getBallCount() {
        return ballCount;
    }

    public boolean hasBallAtPosition(int position) {
        return sensors.get(position).getDistance(DistanceUnit.CM) < MAX_DISTANCE_WHEN_BALL_PRESENT;
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
