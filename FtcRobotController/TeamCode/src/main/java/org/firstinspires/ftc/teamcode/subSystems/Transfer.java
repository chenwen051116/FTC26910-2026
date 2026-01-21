package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer extends SubsystemBase {
    private final Servo transferServo;
    private final Servo upperFeederServo;
    private final Servo lowerFeederServo;

    public Transfer (HardwareMap hardwareMap){
        transferServo = hardwareMap.get(Servo.class, "transfer");
        upperFeederServo = hardwareMap.get(Servo.class, "upperFeeder");
        lowerFeederServo = hardwareMap.get(Servo.class, "lowerFeeder");
    }
}
