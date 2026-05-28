package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.Constants.Shooter.SHORT_RANGE_DISTANCE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.AutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Intake;

abstract class AllianceTeleOp extends LinearOpMode {
    protected abstract boolean isRedAlliance();

    protected Pose initialPose() {
        return null;
    }

    protected void configureBeforeStart(RobotHardware robot) {
    }

    @Override
    public final void runOpMode() {
        RobotHardware robot = new RobotHardware(hardwareMap, gamepad1, gamepad2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(200);

        Pose initialPose = initialPose();
        if (initialPose != null) {
            robot.drivetrain.setLastPose(initialPose);
        }
        robot.drivetrain.initEncoder();
        robot.shooter.initTurretEncoder();
        configureBeforeStart(robot);

        boolean resetPoseButtonWasDown = false;
        boolean shortRangeBurstStarted = false;

        waitForStart();
        while (opModeIsActive()) {
            robot.clearBulkCache();
            boolean isRed = isRedAlliance();

            boolean resetPoseButtonDown = gamepad2.right_trigger > 0.3;
            if (resetPoseButtonDown && !resetPoseButtonWasDown) {
                robot.drivetrain.resetPose(wallResetPose(isRed));
                robot.shooter.resetTurretOffset();
            }
            resetPoseButtonWasDown = resetPoseButtonDown;

            Pose robotPose = robot.drivetrain.getCurrentPose();
            Vector targetDisplacement = robot.shooter.getDisplacement(robot.shooter.getGoalPose(isRed), robotPose);

            robot.ledSet.setBallCount(robot.transfer.getBallCount());
            robot.ledSet.setShooterState(robot.shooter.getShooterState());

            Shooter.ShooterConfig shooterConfig = robot.shooter.calculateShooterConfig(
                    robotPose,
                    robot.drivetrain.getCurrentVelocity(),
                    robot.drivetrain.getCurrentAcceleration(),
                    isRed
            );
            robot.shooter.setShooterConfig(shooterConfig);

            if (robot.shooter.getShooterState() == Shooter.ShooterState.SHOOTING) {
                robot.transfer.overrideDriver();
                robot.transfer.openGate();

                boolean shortRangeShot = isShortRange(targetDisplacement.getMagnitude());
                if (!shortRangeShot) {
                    shortRangeBurstStarted = false;
                }

                if (shortRangeShot && robot.shooter.isAtTargetRPM()) {
                    shortRangeBurstStarted = true;
                }

                if ((shortRangeShot && shortRangeBurstStarted)
                        || (!shortRangeShot && robot.shooter.isAtTargetRPM())) {
                    robot.transfer.feedShooter();
                } else {
                    robot.transfer.setIntakeState(Intake.IntakeState.STOP);
                }
            } else {
                shortRangeBurstStarted = false;
                robot.transfer.stopOverrideDriver();
            }

            robot.drivetrain.periodic();
            robot.transfer.periodic();
            robot.shooter.periodic();
            robot.ledSet.periodic();

            addTelemetry(robot, shooterConfig, targetDisplacement);
        }
    }

    private Pose wallResetPose(boolean isRed) {
        if (isRed) {
            return new Pose(
                    AutoConstants.RedNear.startX,
                    AutoConstants.RedNear.startY,
                    AutoConstants.RedNear.startHeading
                    );
        }

        return new Pose(
                AutoConstants.BlueNear.startX,
                AutoConstants.BlueNear.startY,

                AutoConstants.BlueNear.startHeading
        );
    }

    private boolean isShortRange(double distance) {
        return distance <= SHORT_RANGE_DISTANCE[SHORT_RANGE_DISTANCE.length - 1];
    }

    protected void addTelemetry(RobotHardware robot, Shooter.ShooterConfig shooterConfig, Vector targetDisplacement) {
        telemetry.addData("!!! Current kv", robot.shooter.getCurrentFlywheelKv() * 100000);
        telemetry.addData("!!! Flywheel RPM", robot.shooter.getFlywheelRPM());
        telemetry.addData("!!! Target RPM", shooterConfig.flywheelRPM);
        telemetry.addData("!!! At Target RPM", robot.shooter.isAtTargetRPM());
        telemetry.addData("!!! Short Range", isShortRange(targetDisplacement.getMagnitude()));
        telemetry.addData("!!! Overriding Shooter", robot.shooter.isOverriding());
        telemetry.addData("Follower X", robot.drivetrain.getCurrentPose().getX());
        telemetry.addData("Follower Y", robot.drivetrain.getCurrentPose().getY());
        telemetry.addData("Follower heading", robot.drivetrain.getCurrentPose().getHeading());
        telemetry.addData("Target turret angle", shooterConfig.turretAngle);
        telemetry.addData("Number of balls", robot.transfer.getBallCount());
        telemetry.addData("Intake state", robot.transfer.getIntakeState());
        telemetry.addData("Turret angle", robot.shooter.getTurretAngle());
        telemetry.addData("Turret servo position", robot.shooter.getTurretPower());
        telemetry.addData("Gate open", robot.transfer.isGateOpen());
        telemetry.addData("Flywheel power", robot.shooter.getFlywheelPower());
        telemetry.addData("Hood Position", robot.shooter.getHoodPosition());
        telemetry.addData("Flywheel RPM1", robot.shooter.getFlywheelMotor1RPM());
        telemetry.addData("Flywheel RPM2", robot.shooter.getFlywheelMotor2RPM());
        telemetry.addData("Distance to goal", targetDisplacement.getMagnitude());
        telemetry.update();
    }
}
