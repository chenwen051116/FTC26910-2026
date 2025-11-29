package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.subsystems.AutoDrivetrain.fwd1;


import com.pedropathing.util.Timer;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.AutoDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous
public class FinalAutoSmallTriangleFar extends LinearOpMode {

    // private MultipleTelemetry telemetry;
    private Timer timer;
    private AutoDrivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    //    private Shooter shooter;
    private Limelight limeLight;
    private Turret turret;
    private double x, y, rx;



    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drivetrain = new AutoDrivetrain(hardwareMap);

        intake = new Intake(hardwareMap);
        limeLight = new Limelight(hardwareMap);
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
            return;


        }
    }
}