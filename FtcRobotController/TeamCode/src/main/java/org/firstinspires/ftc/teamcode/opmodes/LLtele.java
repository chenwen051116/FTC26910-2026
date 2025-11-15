package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DriveInTeleOpCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LimelightLockInCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MyLimelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class LLtele extends LinearOpMode {

    //private Telemetry telemetry;
    private MyLimelight limeLight;
    //    private boolean xJustPressed = false;
//    private boolean xHolding = false;
//    private boolean yJustPressed = false;
//    private boolean yHolding = false;
    private double x, y, rx;


    @Override
    public void runOpMode() {

        limeLight = new MyLimelight(hardwareMap);
        limeLight.initRedPipeline();
        limeLight.startDetect();


        waitForStart();
        while (opModeIsActive()){
            limeLight.periodic();
            telemetry.addData("Apriltag dist", limeLight.getDis());
            telemetry.addData("Apriltag X", limeLight.getX());
            telemetry.addData("Apriltag(PoI) Tx", limeLight.getTx());
            telemetry.addData("Apriltag ID", limeLight.getAprilTagID());
            telemetry.addData("Pitch", limeLight.getPitch());

            // shooter
            // telemetry
//            telemetry.addData("Shooter Target RPM", shooter.getTargetRPM());
//            telemetry.addData("Shooter Current RPM", shooter.getFlyWheelRPM());
//            telemetry.addData("PIDoutput", shooter.getCurrentPIDOutput());
//            telemetry.addData("Shooter At Target", shooter.isAtTargetRPM() ? "YES" : "NO");
        }

    }
}