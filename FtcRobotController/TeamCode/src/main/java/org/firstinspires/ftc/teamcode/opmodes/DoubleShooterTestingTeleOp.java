package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subSystems.DoubleShooter;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class DoubleShooterTestingTeleOp extends LinearOpMode{
    private DoubleShooter doubleShooter;
    //private Intake intake = new Intake(hardwareMap);
    //private Limelight limelight = new Limelight(hardwareMap);
    private boolean xJustPressed = false;
    private boolean xHolding = false;
    private boolean yJustPressed = false;
    private boolean yHolding = false;

    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        doubleShooter = new DoubleShooter(hardwareMap);


        telemetry.setMsTransmissionInterval(200);
        waitForStart();
        while (opModeIsActive()){
            doubleShooter.periodic();
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

            // check if up is being hold
            // set shooter status
            // press y to stop the shooter when it is in idle state
            if(yJustPressed && doubleShooter.shooterStatus != DoubleShooter.ShooterStatus.Shooting){
                if(doubleShooter.shooterStatus == DoubleShooter.ShooterStatus.Idling) {
                    doubleShooter.setShooterStatus(DoubleShooter.ShooterStatus.Stop);
                }
                else{
                    doubleShooter.setShooterStatus(DoubleShooter.ShooterStatus.Idling);
                }
                yJustPressed = false;
            }

            // press x to shoot when the shooter is in idle state
            // set the shooter to idle state if the shooter is in shooting state or stop state
            if(xJustPressed){
                if(doubleShooter.shooterStatus == DoubleShooter.ShooterStatus.Idling){
                    doubleShooter.setShooterStatus(DoubleShooter.ShooterStatus.Shooting);
                    //turret.updateAutoShoot(true);
                }
                else{
                    doubleShooter.setShooterStatus(DoubleShooter.ShooterStatus.Idling);
                }
                xJustPressed = false;

            }

            telemetry.addData("Shooter Target RPM", doubleShooter.getTargetRPM());
            telemetry.addData("Shooter Current RPM (BASE ON LEFT)", doubleShooter.getFlyWheelRPM());
            telemetry.addData("Left Shooter RPM", doubleShooter.getLeftWheelRPM());
            telemetry.addData("Right Shooter RPM", doubleShooter.getRightWheelRPM());

            telemetry.update();
        }
    }
}