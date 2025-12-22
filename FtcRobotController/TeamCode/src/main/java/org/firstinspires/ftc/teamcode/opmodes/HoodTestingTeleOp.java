package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Hood;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class HoodTestingTeleOp extends LinearOpMode {
    public Hood hood;

    @Override
    public void runOpMode(){
        hood = new Hood(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            hood.periodic();
            if (gamepad1.dpad_down){
                hood.setHoodAngle(0);
            } else if(gamepad1.dpad_up){
                hood.setHoodAngle(1);
            }

            telemetry.addData("HoodAngle", hood.getHoodAngle());
            telemetry.update();
        }

    }
}
