package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.util.Timer;
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DriveInTeleOpCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LimelightLockInCommand;
import org.firstinspires.ftc.teamcode.subsystems.AutoDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MyLimelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;

@Autonomous
public class FinalAuto extends LinearOpMode {

    // private MultipleTelemetry telemetry;
    private Timer timer;
    private AutoDrivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    //    private Shooter shooter;
    private MyLimelight limeLight;
    private Turret turret;
    private double x, y, rx;

    public static int fwd1 = 1000;
    public static int turn1 = 400;

    public static int fwd2 = 1000;
    public static int turn2 = 400;
    public static int turn3 = 400;
    public static int turn4 = 400;
    public static int fwd3 = 1000;
    public static int fwd4 = 1000;
    public static int fwd5 = 1000;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drivetrain = new AutoDrivetrain(hardwareMap);

        intake = new Intake(hardwareMap);
        limeLight = new MyLimelight(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);

        limeLight.initRedPipeline();
        limeLight.startDetect();
        turret.initEncoder();
        shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
        telemetry.setMsTransmissionInterval(200);
        timer = new Timer();
        waitForStart();
        while (opModeIsActive()) {

            drivetrain.forward(fwd1);
            timer.resetTimer();
            while(timer.getElapsedTimeSeconds()<3){
            }
            drivetrain.turn(turn1);
            timer.resetTimer();
            while(timer.getElapsedTimeSeconds()<3){
            }

            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
            timer.resetTimer();
            while(timer.getElapsedTimeSeconds()<5){
                shooter.periodic();
                //turret.periodic();
                limeLight.periodic();
                intake.periodic();
                intake.updateAutoShoot(true);
                intake.updateAutoTrans(shooter.isAtTargetRPM());
                //intake.updateAutoTrans(true);
                shooter.updateDis(limeLight.getDis());
                shooter.updateFocused(true);
            }
            intake.updateAutoShoot(false);
            shooter.setShooterStatus(Shooter.ShooterStatus.Stop);

            drivetrain.turn(-turn1);
            timer.resetTimer();
            while(timer.getElapsedTimeSeconds()<3){
            }

            drivetrain.forward(fwd2);
            timer.resetTimer();
            while(timer.getElapsedTimeSeconds()<3){
            }
            drivetrain.turn(turn2);
            timer.resetTimer();
            while(timer.getElapsedTimeSeconds()<3){
            }
            intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
            drivetrain.forward(fwd3);
            timer.resetTimer();
            while(timer.getElapsedTimeSeconds()<5){
                intake.periodic();
            }
            intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);

            drivetrain.forward(-fwd4);
            timer.resetTimer();
            while(timer.getElapsedTimeSeconds()<3){
            }
            drivetrain.turn(turn3);
            timer.resetTimer();
            while(timer.getElapsedTimeSeconds()<3){
            }


            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
            timer.resetTimer();
            while(timer.getElapsedTimeSeconds()<5){
                shooter.periodic();
                //turret.periodic();
                limeLight.periodic();
                intake.periodic();
                intake.updateAutoShoot(true);
                intake.updateAutoTrans(shooter.isAtTargetRPM());
                //intake.updateAutoTrans(true);
                shooter.updateDis(limeLight.getDis());
                shooter.updateFocused(true);
            }
            intake.updateAutoShoot(false);
            shooter.setShooterStatus(Shooter.ShooterStatus.Stop);

            drivetrain.turn(turn4);
            timer.resetTimer();
            while(timer.getElapsedTimeSeconds()<3){
            }

            drivetrain.forward(fwd5);
            timer.resetTimer();
            while(timer.getElapsedTimeSeconds()<3){
            }
            return;


        }
    }
}