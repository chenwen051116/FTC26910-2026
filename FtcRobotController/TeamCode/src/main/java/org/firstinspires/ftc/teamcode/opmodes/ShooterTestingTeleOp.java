package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class ShooterTestingTeleOp extends LinearOpMode{
    private Shooter shooter = new Shooter(hardwareMap);
    private Intake intake = new Intake(hardwareMap);
    private Limelight limelight = new Limelight(hardwareMap);
    private boolean xJustPressed = false;
    private boolean xHolding = false;
    private boolean yJustPressed = false;
    private boolean yHolding = false;

    public void runOpMode(){
        telemetry.setMsTransmissionInterval(200);

        while (opModeIsActive()){
            shooter.periodic();
            intake.periodic();
            limelight.periodic();


        }
    }
}