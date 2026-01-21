package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subSystems.Drivetrain;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class ChassisTestingTeleOp extends LinearOpMode {


    private Drivetrain drivetrain;
    private boolean slowMode = false;
    private double x, y, rx;
    private double speedMultiplier = 1;


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drivetrain = new Drivetrain(hardwareMap);
        telemetry.setMsTransmissionInterval(200);
        waitForStart();
        while (opModeIsActive()){
            // drivetrain
            // set drivetrain status

            double x = -gamepad1.left_stick_x * speedMultiplier;
            double y = gamepad1.left_stick_y * speedMultiplier;
            double rx = -gamepad1.right_stick_x * speedMultiplier;
            drivetrain.move(y, x, rx);
            if(gamepad1.left_trigger > 0.3){
                slowMode = true;
            }
            else{
                slowMode = false;
            }

            if (slowMode){
                speedMultiplier = 0.3;
            }
            else{
                speedMultiplier = 1;
            }

            telemetry.update();
        }

    }
}