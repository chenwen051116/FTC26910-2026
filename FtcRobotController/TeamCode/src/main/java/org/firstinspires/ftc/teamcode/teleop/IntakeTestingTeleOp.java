package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Transfer;

@TeleOp(name = "Intake Testing TeleOp")
public class IntakeTestingTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        Transfer transfer = new Transfer(
                gamepad1,
                hardwareMap.get(DcMotorEx.class, RobotHardware.INTAKE_MOTOR_NAME),
                hardwareMap.get(DcMotorEx.class, RobotHardware.TRANSFER_MOTOR_NAME),
                hardwareMap.get(Servo.class, "gate"),
                new DistanceSensor[]{
                        hardwareMap.get(DistanceSensor.class, "distance_sensor_0"),
                        hardwareMap.get(DistanceSensor.class, "distance_sensor_1"),
                        hardwareMap.get(DistanceSensor.class, "distance_sensor_2")
                }
        );

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        waitForStart();
        while (opModeIsActive()) {
            transfer.periodic();

            telemetry.addData("Number of balls", transfer.getBallCount());
            telemetry.addData("Gate open", transfer.isGateOpen());
            telemetry.update();
        }
    }
}
