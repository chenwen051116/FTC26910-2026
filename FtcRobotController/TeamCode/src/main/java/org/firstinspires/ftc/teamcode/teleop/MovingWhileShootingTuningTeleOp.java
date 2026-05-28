package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

@Config
@TeleOp(name = "Red Moving While Shooting Tuning Teleop")
public class MovingWhileShootingTuningTeleOp extends LinearOpMode {
    public static double tuningFlywheelRPM = 3000;
    public static double tuningTurretAngle = 0;
    public static double tuningHoodPosition = 0;

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(hardwareMap, gamepad1, gamepad2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(200);

        robot.drivetrain.setLastPose(new Pose(9, 9, 0));
        robot.drivetrain.initEncoder();
        robot.shooter.initTurretEncoder();

        waitForStart();
        while (opModeIsActive()) {
            robot.clearBulkCache();
            robot.shooter.setShooterConfig(new Shooter.ShooterConfig(
                    tuningTurretAngle,
                    tuningHoodPosition,
                    tuningFlywheelRPM
            ));

            robot.drivetrain.periodic();
            robot.transfer.periodic();
            robot.shooter.periodic();
            robot.ledSet.periodic();

            if (gamepad1.dpadLeftWasPressed()) {
                tuningTurretAngle -= Math.toRadians(5);
            }

            if (gamepad1.dpadRightWasPressed()) {
                tuningTurretAngle += Math.toRadians(5);
            }

            telemetry.addData("Follower X", robot.drivetrain.getCurrentPose().getX());
            telemetry.addData("Follower Y", robot.drivetrain.getCurrentPose().getY());
            telemetry.addData("Follower heading", robot.drivetrain.getCurrentPose().getHeading());
            telemetry.addData("Flywheel RPM", robot.shooter.getFlywheelRPM());
            telemetry.addData("Target RPM", robot.shooter.getShooterConfig().flywheelRPM);
            telemetry.addData("Number of balls", robot.transfer.getBallCount());
            telemetry.addData("Hood Position", robot.shooter.getHoodPosition());
            telemetry.addData("Turret angle", robot.shooter.getTurretAngle());
            telemetry.addData("Turret servo position", robot.shooter.getTurretPower());
            telemetry.addData("Flywheel power", robot.shooter.getFlywheelPower());
            telemetry.addData("Flywheel RPM1", robot.shooter.getFlywheelMotor1RPM());
            telemetry.addData("Flywheel RPM2", robot.shooter.getFlywheelMotor2RPM());
            telemetry.addData("Distance to RED goal",
                    robot.shooter.getDisplacement(robot.shooter.getGoalPose(true), robot.drivetrain.getCurrentPose()).getMagnitude());
            telemetry.update();
        }
    }
}
