package org.firstinspires.ftc.teamcode.subSystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Hook extends SubsystemBase {
    private final Servo hook;
    public HookStates currentHookState = HookStates.standBy;

    public static double hookAngle = 0;


    public enum HookStates{
        push(1),
        standBy(0),
        pull(0.0);
        private final double hookAngle;
        HookStates(double hookAngle){
            this.hookAngle = hookAngle;
        }
    }

    // Constructor for hook motor
    public Hook(HardwareMap hardwareMap){
        hook = hardwareMap.get(Servo.class, "hook");
        hook.setDirection(Servo.Direction.REVERSE);
        hook.setPosition(hookAngle);
    }

    public void setHookState(HookStates hookState){
        currentHookState = hookState;
        setHookAngle(currentHookState.hookAngle);
    }

    public void updateHookAngle(){
        hook.setPosition(hookAngle);
    }

    public double getHookAngle(){
        return hookAngle;
    }

    public void setHookAngle(double angle){
        hookAngle = angle;
    }

    @Override
    public void periodic(){
        updateHookAngle();
    }
}
