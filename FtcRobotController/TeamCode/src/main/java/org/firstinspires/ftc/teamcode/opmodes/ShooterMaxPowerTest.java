package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subSystems.Intake;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class ShooterMaxPowerTest extends LinearOpMode{
    private DcMotorEx leftShooter;
    private DcMotorEx rightShooter;
    private Intake intake;
    //private Intake intake = new Intake(hardwareMap);
    //private Limelight limelight = new Limelight(hardwareMap);
    private boolean xJustPressed = false;
    private boolean xHolding = false;
    private boolean shootingStatus = false;

    private boolean dPadUpHolding = false;
    private boolean dPadUpJustPressed = false;
    private boolean dPadDownHolding = false;
    private boolean dPadDownJustPressed = false;

    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");

        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = new Intake(hardwareMap);
        telemetry.setMsTransmissionInterval(200);
        waitForStart();
        while (opModeIsActive()){
            // check keys
            // check if x is being hold
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

            // check if y is being hold

            // press x to shoot when the shooter is in idle state
            // set the shooter to idle state if the shooter is in shooting state or stop state
            if(xJustPressed){
                if(shootingStatus){
                    shootingStatus = false;
                    leftShooter.setPower(0);
                    rightShooter.setPower(0);
                }
                else{
                    shootingStatus = true;
                    leftShooter.setPower(1);
                    rightShooter.setPower(1);
                }
                xJustPressed = false;

            }

            // check if up is being hold
            if(gamepad1.dpad_up){
                if(!dPadUpHolding){
                    dPadUpJustPressed = true;
                    dPadUpHolding = true;
                }
            }
            else{
                dPadUpHolding = false;
                dPadUpJustPressed = false;
            }

            // check if down is being hold
            if(gamepad1.dpad_down){
                if(!dPadDownHolding){
                    dPadDownJustPressed = true;
                    dPadDownHolding = true;
                }
            }
            else{
                dPadDownHolding = false;
                dPadDownJustPressed = false;
            }


            if (dPadDownHolding){
                intake.setIntakeStatusTo(Intake.IntakeStates.Ball_In);
            }
            else{
                intake.setIntakeStatusTo(Intake.IntakeStates.Stop);
            }

            if (dPadUpHolding){
                intake.setIntakeStatusTo(Intake.IntakeStates.Send_Ball);
            }
            else{
                intake.setIntakeStatusTo(Intake.IntakeStates.Stop);
            }

            telemetry.addData("Left Shooter RPM", leftShooter.getVelocity() * 60.0 / 28.0);
            telemetry.addData("Right Shooter RPM", rightShooter.getVelocity() * 60.0 / 28.0);

            intake.periodic();
            telemetry.update();
        }
    }
}