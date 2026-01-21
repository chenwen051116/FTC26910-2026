package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subSystems.Drivetrain;

public class DriveInTeleOpCommand extends CommandBase {
    Drivetrain drivetrain;
    Gamepad gamepad1;

    public DriveInTeleOpCommand (Gamepad gamepad1, Drivetrain drivetrain) { //()里传参
        this.drivetrain = drivetrain; //this. = instance variable(上面的), 右面的 = ()里的
        this.gamepad1 = gamepad1;
        addRequirements(drivetrain);
    }

    @Override //Annotation, 重写super class的函数
    public void initialize() {}

    @Override
    public void execute() { //scheduler periodically calls the function
            drivetrain.move(-0.9*gamepad1.left_stick_y, 0.9*gamepad1.left_stick_x, 0.7*gamepad1.right_stick_x);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {

        return false;
    }
}
