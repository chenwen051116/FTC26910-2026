package org.firstinspires.ftc.teamcode.old.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.old.subSystems.Intake;

public class IntakeCommand extends CommandBase {
    Intake intake;
    Gamepad gamepad1;

    public IntakeCommand (Gamepad gamepad1, Intake intake) { //()里传参
        this.intake = intake; //this. = instance variable(上面的), 右面的 = ()里的
        this.gamepad1 = gamepad1;
        addRequirements(intake);
    }

    public void execute() {
        if (gamepad1.right_trigger > 0.3 ){
        intake.setIntakeStatusTo(Intake.IntakeStates.Intake);
    }else if (gamepad1.left_trigger > 0.3){
            intake.setIntakeStatusTo(Intake.IntakeStates.Outtake);
    }else if (gamepad1.right_bumper) {
            intake.setIntakeStatusTo(Intake.IntakeStates.Feeding);
    }else {
            intake.setIntakeStatusTo(Intake.IntakeStates.Stop);
    }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakePowerTo(0);
    }
}
