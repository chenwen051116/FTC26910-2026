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
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class FinalTele extends LinearOpMode {

   // private MultipleTelemetry telemetry;

    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    //    private Shooter shooter;
    private MyLimelight limeLight;
    private Turret turret;
        private boolean xJustPressed = false;
    private boolean xHolding = false;
    private boolean yJustPressed = false;
    private boolean yHolding = false;
    private double x, y, rx;


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drivetrain = new Drivetrain(hardwareMap);

        intake = new Intake(hardwareMap);
        limeLight = new MyLimelight(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);

        limeLight.initRedPipeline();
        limeLight.startDetect();
        turret.initEncoder();
        shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
        telemetry.setMsTransmissionInterval(200);
        waitForStart();
        while (opModeIsActive()){
            shooter.periodic();
            turret.periodic();
            limeLight.periodic();

            if(shooter.shooterStatus == Shooter.ShooterStatus.Shooting){
                intake.updateAutoShoot(true);
                //intake.updateAutoTrans(shooter.isAtTargetRPM());
                intake.updateAutoTrans(true);
                shooter.updateDis(limeLight.getDis());
                shooter.updateFocused(limeLight.isFocused());
                turret.tx = limeLight.getTx();
                turret.updateAutoShoot(true);
            }
            else{
                intake.updateAutoShoot(false);
                turret.updateAutoShoot(false);
            }
            // check keys
            if(gamepad1.x){
                if(!xHolding){
                    xJustPressed = true;
                    xHolding = true;
                }
            }
            else{
                xHolding = false;
                xJustPressed = false;
            }

            if(gamepad1.y){
                if(!yHolding){
                    yJustPressed = true;
                    yHolding = true;
                }
            }
            else{
                yHolding = false;
                yJustPressed = false;
            }

            // set shooter status
            if(yJustPressed &&shooter.shooterStatus != Shooter.ShooterStatus.Shooting){
                if(shooter.shooterStatus == Shooter.ShooterStatus.Idling) {
                    shooter.setShooterStatus(Shooter.ShooterStatus.Stop);
                }
                else{
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                }
                yJustPressed = false;
            }
            if(xJustPressed){
                if(shooter.shooterStatus == Shooter.ShooterStatus.Shooting){
                    shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
                }
                else{
                    shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
                }
                xJustPressed = false;

            }


            // drivetrain
            // set drivetrain status

            double x = -gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x;
            drivetrain.teleDrive(y, x, rx);

            // intake
            // set intake status
            if (gamepad1.right_trigger > 0.3 ){
                intake.setIntakeState(Intake.IntakeTransferState.Suck_In);
            } else if (gamepad1.left_trigger > 0.3){
                intake.setIntakeState(Intake.IntakeTransferState.Split_Out);
            } else if (gamepad1.right_bumper) {
                intake.setIntakeState(Intake.IntakeTransferState.Send_It_Up);
            } else {
                intake.setIntakeState(Intake.IntakeTransferState.Intake_Steady);
            }


            // limelight
            // telemetry
            telemetry.addData("Apriltag dist", limeLight.getDis());
            telemetry.addData("Apriltag X", limeLight.getX());
            telemetry.addData("Apriltag(PoI) Tx", limeLight.getTx());
            telemetry.addData("Apriltag ID", limeLight.getAprilTagID());
            telemetry.addData("Pitch", limeLight.getPitch());

            // shooter
            // telemetry
            telemetry.addData("Turret pos", turret.currentpos);
            telemetry.addData("Shooter Target RPM", shooter.getTargetRPM());
            telemetry.addData("Shooter Current RPM", shooter.getFlyWheelRPM());
            telemetry.addData("PIDoutput", shooter.getCurrentPIDOutput());
            telemetry.addData("Shooter At Target", shooter.isAtTargetRPM() ? "YES" : "NO");
            telemetry.update();
        }

    }
}