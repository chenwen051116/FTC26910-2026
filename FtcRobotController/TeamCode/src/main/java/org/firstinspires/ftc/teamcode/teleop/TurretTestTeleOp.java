package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Turret;

@Config
@TeleOp(name = "Turret Test TeleOp")
public class TurretTestTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        Turret turret = new Turret(
                hardwareMap.get(Servo.class, RobotHardware.TURRET_PRIMARY_SERVO_NAME),
                hardwareMap.get(Servo.class, RobotHardware.TURRET_SECONDARY_SERVO_NAME)
        );

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        turret.zero();

        waitForStart();
        turret.zero();

        while (opModeIsActive()) {
            turret.manualControl(gamepad1.left_stick_x);

            telemetry.addData("Left stick X", gamepad1.left_stick_x);
            telemetry.addData("Turret target angle", Math.toDegrees(turret.getTargetAngle()));
            telemetry.addData("Turret angle", Math.toDegrees(turret.getCurrentAngle()));
            telemetry.addData("Servo position", turret.getPosition());
            telemetry.update();
        }
    }
}
