package org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Overridable;

@Config
public class Drivetrain extends Overridable {
    private final DcMotor frontLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor backRightMotor;
    private final Gamepad gamepad;
    private final Follower follower;
    public static double regularSpeedMultiplier = 1;
    public static double slowSpeedMultiplier = 0.3;

    public Drivetrain(Gamepad gamepad, DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor, Follower follower) {
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor ;

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

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

        stopOverrideDriver();
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

    public void initEncoder(Pose lastPose){
        follower.startTeleopDrive();
        follower.update();
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.setPose(lastPose);
    }

    public Pose getCurrentPose(){
        return follower.getPose();
    }

    public Vector getCurrentVelocity(){
        return follower.getVelocity();
    }

    public Vector getCurrentAcceleration(){
        return follower.getAcceleration();
    }

    public void goTo(Pose pose) {
        if (!isOverriding() || follower.isBusy()) {
            return;
        }

        Pose currentPose = follower.getPose();

        follower.followPath(
                follower.pathBuilder()
                        .addPath(new BezierLine(currentPose, pose))
                        .setLinearHeadingInterpolation(currentPose.getHeading(), pose.getHeading())
                        .build()
        );
    }
    @Override
    public void runWithoutOverride() {
        double x = -gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double rx = -gamepad.right_stick_x;

        double frontLeftPower = y - x - rx;
        double frontRightPower = y + x + rx;
        double backLeftPower = y + x - rx;
        double backRightPower = y - x + rx;

        double speedMultiplier;
        if (gamepad.left_trigger > 0.3){
            speedMultiplier = slowSpeedMultiplier;
        } else{
            speedMultiplier = regularSpeedMultiplier;
        }
        frontLeftMotor.setPower(frontLeftPower * speedMultiplier);
        frontRightMotor.setPower(frontRightPower * speedMultiplier);
        backLeftMotor.setPower(backLeftPower * speedMultiplier);
        backRightMotor.setPower(backRightPower * speedMultiplier);
    }

    @Override
    public void alwaysRunning() {
        follower.updatePose();
    }
}
