package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Hood extends SubsystemBase {
    private final Servo hood;

    public static double hoodAngle = 0;

    public Hood(HardwareMap hardwareMap){
        hood = hardwareMap.get(Servo.class, "Hood");
        hood.setPosition(hoodAngle);
    }

    public void updateHoodAngle(){
        hood.setPosition(hoodAngle);
    }

    public double getHoodAngle(){
        return hoodAngle;
    }

    public void setHoodAngle(double angle){
        if (angle<0) {
            angle = 0;
        }
        else if (angle>1){
            angle = 1;
        }
        hoodAngle = angle;
    }

    @Override
    public void periodic(){
        updateHoodAngle();
    }
}
