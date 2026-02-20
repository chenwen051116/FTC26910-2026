package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.I2cDevice;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain.Drivetrain;

@TeleOp(name = "EncoderTestingTeleOp")
public class EncoderTestingTeleOp extends LinearOpMode {
    GoBildaPinpointDriver encoder;

    private DcMotorEx getMotor(String motorName) {
        return hardwareMap.get(DcMotorEx.class, motorName);
    }

    public void runOpMode() {
        encoder = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(200);
        encoder.resetPosAndIMU();



        DcMotor frontLeftMotor = getMotor("front_left");
        DcMotor frontRightMotor = getMotor("front_right");
        DcMotor backLeftMotor = getMotor("back_left");
        DcMotor backRightMotor = getMotor("back_right");

        Drivetrain drivetrain = new Drivetrain(gamepad1, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, Constants.createFollower(hardwareMap));

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("ENCODER X", encoder.getEncoderX());
            telemetry.addData("ENCODER Y", encoder.getEncoderY());
            telemetry.addData("ENCODER POS X", encoder.getPosX(DistanceUnit.INCH));
            telemetry.addData("ENCODER POS Y", encoder.getPosY(DistanceUnit.INCH));
            telemetry.addData("HEADING", encoder.getHeading(AngleUnit.DEGREES));
            encoder.update();
            drivetrain.periodic();
            telemetry.update();
        }
    }
}
