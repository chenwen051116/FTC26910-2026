package org.firstinspires.ftc.teamcode.old.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.old.subSystems.Drivetrain;
import org.firstinspires.ftc.teamcode.old.subSystems.Limelight;

@Config
public class LimelightLockInCommand extends CommandBase {
    Drivetrain drivetrain;
    Limelight limelight;
    Gamepad gamepad1;
    public static double Kp = 0.018;

    public LimelightLockInCommand(Drivetrain drivetrain, Limelight limelight, Gamepad gamepad1) { //()里传参
        this.drivetrain = drivetrain; //this. = instance variable(上面的), 右面的 = ()里的
        this.limelight = limelight;
        this.gamepad1 = gamepad1;
        addRequirements(drivetrain, limelight);
    }

    @Override // Annotation, 重写super class的函数
    public void initialize() {
        gamepad1.rumble(200);
        limelight.startDetect();
    }

    @Override
    public void execute() { // scheduler periodically calls the function
        drivetrain.move(-0.9 * gamepad1.left_stick_y, 0.9 * gamepad1.left_stick_x,
                Kp * limelight.getTx());

    }

    @Override
    public void end(boolean interrupted) {
        gamepad1.rumble(200);
        limelight.stopDetect();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}