package org.firstinspires.ftc.teamcode.subSystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class DistSensor extends SubsystemBase {
    private final DistanceSensor firstSensor;
//    private final DistanceSensor rightSensor;
    public double distance;
    public static double containsBallDistance = 4.0;
    public boolean containsFirstBall = false;

    // Constructor for hood motor
    public DistSensor(HardwareMap hardwareMap){
        firstSensor = hardwareMap.get(DistanceSensor.class, "distanceSensorFirst");
//        rightSensor = hardwareMap.get(DistanceSensor.class, "distanceSensorRight");
        distance = 0;
    }

    public double getFirstSensorDistanceCM(){
        return firstSensor.getDistance(DistanceUnit.CM);
    }
//    public double getRightSensorDistanceCM(){
//        return rightSensor.getDistance(DistanceUnit.CM);
//    }

    public boolean containsFirstBall(){
        return containsFirstBall;
    }

    public void updateFirstBallStatus(){
        containsFirstBall = getFirstSensorDistanceCM() <= containsBallDistance;
    }

    @Override
    public void periodic(){
        updateFirstBallStatus();
    }
}
