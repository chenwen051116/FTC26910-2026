package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Hood;

public class HoodTestingTeleOp extends LinearOpMode {
    public Hood hood;

    @Override
    public void runOpMode(){
        hood.periodic();
        if (gamepad1.dpad_down){
            hood.setHoodAngle(0);
        } else if(gamepad1.dpad_up){
            hood.setHoodAngle(1);
        }

    }
}
