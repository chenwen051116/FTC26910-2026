package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.Pose2d;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Pinpoint Odometry Example", group = "Examples")
public class PinpointOdometryExample extends LinearOpMode {

    // Declare hardware
    public PinpointLocalizer pin;
    @Override
    public void runOpMode() {
        // Initialize hardware
        pin = new PinpointLocalizer(hardwareMap,71.0 / 50781, new Pose2d(0, 0, 0));


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read encoder values
            pin.update();

            // Display telemetry
            telemetry.addData("x:", pin.getPose().position.x);
            telemetry.addData("y:", pin.getPose().position.y);
            telemetry.addData("heading:", pin.getPose().heading);

            telemetry.update();
        }
    }
}