package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.util.Timer;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.AutoDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MyLimelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous
@Config
public class FinalAutoRed extends LinearOpMode {

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