package org.firstinspires.ftc.teamcode.subSystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class DistSensor extends SubsystemBase {
    private final DistanceSensor firstSensor;
    private final DistanceSensor secondSensor;
    private final DistanceSensor thirdSensor;
    public double distance;
    public static double containsBallDistance = 4.0;
    public boolean containsFirstBall = false;
    public boolean containsSecondBall = false;
    public boolean containsThirdBall = false;
    public int currentBallCount = 0;

    // Constructor for hood motor
    public DistSensor(HardwareMap hardwareMap){
        firstSensor = hardwareMap.get(DistanceSensor.class, "distanceSensorFirst");
        secondSensor = hardwareMap.get(DistanceSensor.class, "distanceSensorSecond");
        thirdSensor = hardwareMap.get(DistanceSensor.class, "distanceSensorThird");
        distance = 0;
    }

    public double getFirstSensorDistanceCM(){
        return firstSensor.getDistance(DistanceUnit.CM);
    }

    public double getSecondSensorDistanceCM(){
        return secondSensor.getDistance(DistanceUnit.CM);
    }

    public double getThirdSensorDistanceCM(){
        return thirdSensor.getDistance(DistanceUnit.CM);
    }

    public boolean containsFirstBall(){
        return containsFirstBall;
    }

    public boolean containsSecondBall(){
        return containsSecondBall;
    }

    public boolean ContainsThirdBall(){
        return containsThirdBall;
    }

    public void updateBallStates(){
        containsFirstBall = getFirstSensorDistanceCM() <= containsBallDistance;
        containsSecondBall = getSecondSensorDistanceCM() <= containsBallDistance;
        containsThirdBall = getThirdSensorDistanceCM() <= containsBallDistance;
    }

    public void updateCurrentBallNumber(){
        if (containsFirstBall && containsSecondBall && containsThirdBall){
            currentBallCount = 3;
        } else if (containsFirstBall && containsSecondBall){
            currentBallCount = 2;
        } else if (containsFirstBall){
            currentBallCount = 1;
        } else{
            currentBallCount = 0;
        }
    }

    public int getCurrentBallCount(){
        return currentBallCount;
    }

    @Override
    public void periodic(){
        updateBallStates();
        updateCurrentBallNumber();
    }
}
