package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.AutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Intake;

@Config
@TeleOp(name = "Red Shooter Testing Teleop")
public class ShooterTestingTeleOp extends LinearOpMode {
    public static double tuningFlywheelRPM = 3000;
    public static double tuningTurretAngle = 0;
    public static double tuningHoodPosition = 0;

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(hardwareMap, gamepad1, gamepad2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        robot.drivetrain.setLastPose(new Pose(9, 9, 0));
        robot.drivetrain.initEncoder();
        robot.shooter.initTurretEncoder();

        boolean resetPoseButtonWasDown = false;
        boolean isShooting = false;

        waitForStart();
        while (opModeIsActive()) {
            robot.clearBulkCache();
            robot.shooter.setShooterConfig(new Shooter.ShooterConfig(
                    tuningTurretAngle,
                    tuningHoodPosition,
                    tuningFlywheelRPM
            ));

            boolean resetPoseButtonDown = gamepad2.right_trigger > 0.3;
            if (resetPoseButtonDown && !resetPoseButtonWasDown) {
                robot.drivetrain.resetPose(new Pose(AutoConstants.RedNear.startX,
                        AutoConstants.RedNear.startY,
                        AutoConstants.RedNear.startHeading));
                robot.shooter.resetTurretOffset();
            }
            resetPoseButtonWasDown = resetPoseButtonDown;


            if (robot.shooter.getShooterState() == Shooter.ShooterState.SHOOTING) {
                robot.transfer.overrideDriver();
                isShooting = isShooting || (robot.shooter.isAtTargetRPM() && robot.transfer.isGateOpen());
                if (isShooting) {
                    robot.transfer.setIntakeState(Intake.IntakeState.TRANSFER);
                }
            } else {
                isShooting = false;
                robot.transfer.stopOverrideDriver();
            }

            robot.drivetrain.periodic();
            robot.transfer.periodic();
            robot.shooter.periodic();
            robot.ledSet.periodic();

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
            telemetry.addData("Distance to RED goal (TURRET)",
                    robot.shooter.getDisplacement(robot.shooter.getGoalPose(true), robot.drivetrain.getCurrentPose()).getMagnitude());
            telemetry.update();
        }
    }
}
