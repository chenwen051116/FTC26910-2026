package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Intake;

@Config
@TeleOp(name = "Red Chassis Tracing With PID Teleop")
public class ChassisTracingWithPIDTestTeleOp extends LinearOpMode {
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
            Shooter.ShooterConfig shooterConfig = robot.shooter.calculateShooterConfig(
                    robot.drivetrain.getCurrentPose(),
                    robot.drivetrain.getCurrentVelocity(),
                    robot.drivetrain.getCurrentAcceleration(),
                    true
            );
            robot.shooter.setShooterConfig(shooterConfig);
            robot.ledSet.setBallCount(robot.transfer.getBallCount());
            robot.ledSet.setShooterState(Shooter.ShooterState.OFF);

            if (robot.shooter.getShooterState() == Shooter.ShooterState.SHOOTING) {
                robot.transfer.overrideDriver();
                if (robot.shooter.isAtTargetRPM() && robot.transfer.isGateOpen()) {
                    robot.transfer.setIntakeState(Intake.IntakeState.TRANSFER);
                }
            } else {
                robot.transfer.stopOverrideDriver();
            }

            robot.drivetrain.periodic();
            robot.transfer.periodic();
            robot.shooter.periodic();
            robot.ledSet.periodic();

            Vector targetDisplacement = robot.shooter.getDisplacement(
                    robot.shooter.getGoalPose(true),
                    robot.drivetrain.getCurrentPose()
            );
            telemetry.addData("Follower X", robot.drivetrain.getCurrentPose().getX());
            telemetry.addData("Follower Y", robot.drivetrain.getCurrentPose().getY());
            telemetry.addData("Follower heading", robot.drivetrain.getCurrentPose().getHeading());
            telemetry.addData("Flywheel RPM", robot.shooter.getFlywheelRPM());
            telemetry.addData("Target turret angle", shooterConfig.turretAngle);
            telemetry.addData("Number of balls", robot.transfer.getBallCount());
            telemetry.addData("Turret angle", robot.shooter.getTurretAngle());
            telemetry.addData("Turret servo position", robot.shooter.getTurretPower());
            telemetry.addData("Flywheel power", robot.shooter.getFlywheelPower());
            telemetry.addData("Hood Position", robot.shooter.getHoodPosition());
            telemetry.addData("Target RPM", robot.shooter.getShooterConfig().flywheelRPM);
            telemetry.addData("Flywheel RPM1", robot.shooter.getFlywheelMotor1RPM());
            telemetry.addData("Flywheel RPM2", robot.shooter.getFlywheelMotor2RPM());
            telemetry.addData("Distance to RED goal", targetDisplacement.getMagnitude());
            telemetry.update();
        }
    }
}
