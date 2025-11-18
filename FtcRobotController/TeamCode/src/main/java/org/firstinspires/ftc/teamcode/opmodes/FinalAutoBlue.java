package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.subsystems.AutoDrivetrain.fwd1;
import static org.firstinspires.ftc.teamcode.subsystems.AutoDrivetrain.stf1;


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
public class FinalAutoBlue extends LinearOpMode {

    // private MultipleTelemetry telemetry;
    private Timer timer;
    private AutoDrivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    //    private Shooter shooter;
    private MyLimelight limeLight;
    private Turret turret;
    private double x, y, rx;



    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drivetrain = new AutoDrivetrain(hardwareMap);

        intake = new Intake(hardwareMap);
        limeLight = new MyLimelight(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);

        limeLight.initBluePipeline();
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

            shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
            drivetrain.changemode();
            timer.resetTimer();
            while(timer.getElapsedTimeSeconds()<5){
                drivetrain.focus(limeLight.getTx());
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
//
            drivetrain.strafe(-stf1);
            timer.resetTimer();
            while(timer.getElapsedTimeSeconds()<1){
            }
            return;


        }
    }
}