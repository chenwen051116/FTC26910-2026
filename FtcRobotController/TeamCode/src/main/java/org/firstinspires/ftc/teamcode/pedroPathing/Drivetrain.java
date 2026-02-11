package org.firstinspires.ftc.teamcode.pedroPathing;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain extends SubsystemBase {
    private final DcMotor frontLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor backRightMotor;
    private final Gamepad gamepad;
    private final Follower follower;
    private Pose currentPose;
    private boolean isOverrideDriver;

    public Drivetrain(HardwareMap hardwareMap, Gamepad gamepad, Follower follower) {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.gamepad = gamepad;
        this.follower = follower;

        isOverrideDriver = false;
    }

    public double getFrontLeftPower() {
        return frontLeftMotor.getPower();
    }

    public double getFrontRightPower() {
        return frontRightMotor.getPower();
    }

    public double getBackLeftPower() {
        return backLeftMotor.getPower();
    }

    public double getBackRightPower() {
        return backRightMotor.getPower();
    }

    public void goTo(Pose pose) {
        if (!isOverrideDriver || follower.isBusy()) {
            return;
        }

        follower.followPath(
                follower.pathBuilder()
                        .addPath(new BezierLine(currentPose, pose))
                        .setLinearHeadingInterpolation(currentPose.getHeading(), pose.getHeading())
                        .build()
        );
    }

    public void periodic() {
        if (isOverrideDriver) {
            follower.update();
        } else {
            double x = gamepad.left_stick_x;
            double y = gamepad.left_stick_y;
            double rx = gamepad.right_stick_x;

            double frontLeftPower = y - x + rx;
            double frontRightPower = y + x + rx;
            double backLeftPower = y + x - rx;
            double backRightPower = y - x - rx;

            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backLeftMotor.setPower(backLeftPower);
            backRightMotor.setPower(backRightPower);

            follower.updatePose();
            currentPose = follower.getPose();
        }
    }
}
