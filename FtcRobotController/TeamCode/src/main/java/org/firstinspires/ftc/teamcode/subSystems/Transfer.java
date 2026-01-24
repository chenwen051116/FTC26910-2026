package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer extends SubsystemBase {

    //private
    private final Servo transferServo;
    private final Servo upperFeederServo;
    private final Servo lowerFeederServo;

    private static double position;

    public Transfer (HardwareMap hardwareMap){
        transferServo = hardwareMap.get(Servo.class, "transfer");
        upperFeederServo = hardwareMap.get(Servo.class, "upperFeeder");
        lowerFeederServo = hardwareMap.get(Servo.class, "lowerFeeder");

        transferServo.setDirection(Servo.Direction.FORWARD);
        upperFeederServo.setDirection(Servo.Direction.FORWARD);
        lowerFeederServo.setDirection(Servo.Direction.FORWARD);

        setPosition(0);
    }

    // temporary method

    public void setPosition(double pos){
        position = pos;
    }

    public void updatePosition(){
        transferServo.setPosition(position);
        upperFeederServo.setPosition(position);
        lowerFeederServo.setPosition(position);
    }

    @Override
    public void periodic(){
        updatePosition();
    }



}
