package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subSystems.DistSensor;
import org.firstinspires.ftc.teamcode.subSystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.Intake.IntakeShooterStates;
import org.firstinspires.ftc.teamcode.subSystems.LEDIndicator;
import org.firstinspires.ftc.teamcode.subSystems.LEDIndicator.LEDShooterStatus;
import org.firstinspires.ftc.teamcode.subSystems.Limelight;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.Shooter.ShooterStates;
import org.firstinspires.ftc.teamcode.subSystems.Turret;
import org.firstinspires.ftc.teamcode.subSystems.Turret.TurretShooterStates;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

   // private MultipleTelemetry telemetry;

    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Limelight limelight;
    private DistSensor distSensor;
    private LEDIndicator ledIndicator;
    private Turret turret;
    private boolean xJustPressed = false;
    private boolean xHolding = false;
    private boolean yJustPressed = false;
    private boolean yHolding = false;
    private boolean aJustPressed = false;
    private boolean aHolding = false;
    private boolean bJustPressed = false;
    private boolean bHolding = false;
    private boolean slowMode = false;
    private boolean dPadUpHolding = false;
    private boolean dPadUpJustPressed = false;
    private boolean dPadDownHolding = false;
    private boolean dPadDownJustPressed = false;
    private double x, y, rx;
    private double speedMultiplier = 1;
    private double prevLimelightDistance = 0;
    private int limelightState = -1;

    @Override
    public void runOpMode() {
        // Initialize instances
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drivetrain = new Drivetrain(hardwareMap);
        ledIndicator = new LEDIndicator(hardwareMap);
        intake = new Intake(hardwareMap);
        limelight = new Limelight(hardwareMap);
        shooter = new Shooter(hardwareMap);
        distSensor = new DistSensor(hardwareMap);
        turret = new Turret(hardwareMap);

        // default red
        limelight.initRedPipeline();
        limelight.startDetect();
        shooter.setShooterStatusTo(Shooter.ShooterStates.Stop);
        telemetry.setMsTransmissionInterval(200);
        waitForStart();
        while (opModeIsActive()){
            // Load periodic functions
            shooter.periodic();
            turret.periodic();
            limelight.periodic();
            intake.periodic();
            distSensor.periodic();
            ledIndicator.periodic();

            // update turret current position using encoder
            turret.setCurrentPosTo(intake.getEncoderValue());

            // update intake status if it has first ball
            intake.setFirstBallStatusTo(!distSensor.containsFirstBall());

            // store limelight distance if not detected
            if(limelight.getDis() != 0){
                prevLimelightDistance = limelight.getDis();
            }

            // Update shooter status
            if(shooter.isAtShooterState(ShooterStates.Shooting)){
                intake.setShooterStatusTo(IntakeShooterStates.Shooting);
                intake.setShooterIsAtTargetRPMStatusTo(shooter.isAtTargetRPM());
                shooter.updateTargetDistance(prevLimelightDistance);
                turret.tx = limelight.getTx();
                turret.setShooterStatusTo(TurretShooterStates.Shooting);
                ledIndicator.setShooterStatusTo(true);
                ledIndicator.setOnTarget(limelight.onTarget());
            } else if (shooter.isAtShooterState(ShooterStates.BurstShooting)){
                intake.setShooterStatusTo(IntakeShooterStates.Burst_Shooting);
                intake.setShooterIsAtTargetRPMStatusTo(shooter.burstShooting);
                turret.tx = limelight.getTx();
                turret.setShooterStatusTo(TurretShooterStates.Shooting);
                ledIndicator.setShooterStatusTo(true);
                ledIndicator.setOnTarget(limelight.onTarget());
            }
            else{
                intake.setShooterStatusTo(IntakeShooterStates.Off);
                turret.setShooterStatusTo(TurretShooterStates.Off);
                ledIndicator.setShooterStatusTo(false);
            }

            // Update LED color based on ball count
            ledIndicator.updateBallCount(distSensor.getCurrentBallCount());

            // Update rear LED indicator based on shooter status
            if(shooter.isAtShooterState(ShooterStates.Shooting) || shooter.isAtShooterState(ShooterStates.BurstShooting)){
                ledIndicator.setShooterStatusTo(LEDShooterStatus.Shooting);
            } else if (shooter.isAtShooterState(ShooterStates.Idling)){
                ledIndicator.setShooterStatusTo(LEDShooterStatus.Idle);
            } else{
                ledIndicator.setShooterStatusTo(LEDShooterStatus.Off);
            }

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

            // check if a is being hold
            if(gamepad1.a){
                if(!aHolding){
                    aJustPressed = true;
                    aHolding = true;
                }
            }
            else{
                aHolding = false;
                aJustPressed = false;
            }

            // check if b is being hold
            if(gamepad1.b){
                if(!bHolding){
                    bJustPressed = true;
                    bHolding = true;
                }
            }
            else{
                bHolding = false;
                bJustPressed = false;
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

            // set shooter status
            // press y to stop the shooter when it is in idle state
            if(yJustPressed && !shooter.isAtShooterState(ShooterStates.Shooting) && !shooter.isAtShooterState(ShooterStates.BurstShooting)){
                if(shooter.isAtShooterState(ShooterStates.Idling)) {
                    shooter.setShooterStatusTo(ShooterStates.Stop);
                }
                else{
                    shooter.setShooterStatusTo(ShooterStates.Idling);
                }
                yJustPressed = false;
            }

            // press x to shoot when the shooter is in idle state
            // set the shooter to idle state if the shooter is in shooting state or stop state
            if(xJustPressed){
                if(shooter.isAtShooterState(ShooterStates.Idling)){
                    prevLimelightDistance = 0;
                    shooter.setShooterStatusTo(ShooterStates.Shooting);
                }
                else if (shooter.isAtShooterState(ShooterStates.Shooting)) {
                    shooter.setShooterStatusTo(ShooterStates.Idling);
                }
                xJustPressed = false;
            }

            // press a to set to manual shooting mode when the shooter is in idle state
            // set the shooter to idle state if the shooter is in manual shooting state or stop state
            if(aJustPressed){
                if (shooter.isAtShooterState(ShooterStates.Idling)){
                    prevLimelightDistance = 0;
                    shooter.setShooterStatusTo(ShooterStates.BurstShooting);
                }
                else if (shooter.isAtShooterState(ShooterStates.BurstShooting)){
                    shooter.setShooterStatusTo(ShooterStates.Idling);
                }
                aJustPressed = false;
            }

            // drivetrain
            // set drivetrain status

            x = gamepad1.left_stick_x * speedMultiplier;
            y = gamepad1.left_stick_y * speedMultiplier;
            rx = -gamepad1.right_stick_x * speedMultiplier;
            drivetrain.move(y, x, rx);
            if(gamepad1.left_bumper){
                turret.tx = limelight.getTx();
                turret.setShooterStatusTo(TurretShooterStates.Shooting);
                ledIndicator.setShooterStatusTo(true);
                ledIndicator.setOnTarget(limelight.onTarget());
            }
            else{
                if(shooter.shooterStatus != Shooter.ShooterStates.Shooting){
                    turret.setShooterStatusTo(TurretShooterStates.Off);
                    ledIndicator.setShooterStatusTo(false);
                }
            }

            if(gamepad1.left_trigger > 0.3){
                slowMode = true;
            }
            else{
                slowMode = false;
            }

            if (slowMode){
                speedMultiplier = 0.3;
            }
            else{
                speedMultiplier = 1;
            }

            // intake
            // set intake status
            if (gamepad1.right_trigger > 0.3){
                intake.setIntakeStatusTo(Intake.IntakeStates.Intake);
            } else if (gamepad1.dpad_up){
                intake.setIntakeStatusTo(Intake.IntakeStates.Outtake);
            } else if (gamepad1.right_bumper) {
                intake.setIntakeStatusTo(Intake.IntakeStates.Feeding);
            } else {
                intake.setIntakeStatusTo(Intake.IntakeStates.Stop);
            }


            // limelight
            // press left to initiate as red, right to initiate as blue
            // changable during game
            if (gamepad1.dpad_left){
                limelightState = -1;
                limelight.initRedPipeline();
                limelight.startDetect();
            }
            else if(gamepad1.dpad_right){
                limelightState = 1;
                limelight.initBluePipeline();
                limelight.startDetect();
            }


            // telemetry
            if (limelightState == 1){
                telemetry.addData("Detecting for color", "BLUE");
            } else if (limelightState == -1){
                telemetry.addData("Detecting for color", "RED");
            } else{
                telemetry.addData("Detecting for color", "NONE");
            }
            telemetry.addData("Apriltag dist", limelight.getDis());
            telemetry.addData("Apriltag X", limelight.getX());
            telemetry.addData("Apriltag(PoI) Tx", limelight.getTx());
            telemetry.addData("Apriltag ID", limelight.getAprilTagID());
            telemetry.addData("Pitch", limelight.getPitch());
            telemetry.addData("Turret pos", turret.currentPos);
            telemetry.addData("Shooter Target RPM", shooter.getTargetRPM());
            telemetry.addData("Shooter Current RPM", shooter.getFlyWheelRPM());
            telemetry.addData("PIDoutput", shooter.getCurrentMotorPIDOutput());
            telemetry.addData("Shooter At Target", shooter.isAtTargetRPM() ? "YES" : "NO");
            telemetry.addData("Hood Angle", shooter.getCurrentHoodAngle());
            telemetry.addData("PID", shooter.getCurrentMotorPIDOutput());
            telemetry.addData("MOTOR", shooter.getCurrentMotorPower());
            telemetry.addData("Hood position", shooter.getCurrentHoodAngle());
            telemetry.addData("Hood is at position", shooter.hoodIsAtTargetPosition());
            telemetry.addData("Burst Shooting", shooter.burstShooting);
            telemetry.addData("Burst Shooting begin", shooter.beginBurstShooting);
            telemetry.addData("First Dist Sensor in CM", distSensor.getFirstSensorDistanceCM());
            telemetry.addData("Servo power display", turret.currentPower);
            telemetry.addData("Limelight focus power", turret.limelightFocusPower);
            telemetry.addData("Limelight On target", limelight.onTarget());
            telemetry.addData("LED shooting state", ledIndicator.shootingStatus);
            telemetry.update();
        }

    }
}