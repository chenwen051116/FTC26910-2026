package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Limelight;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class LimelightTestingTeleOp extends LinearOpMode {

    //private Telemetry telemetry;
    private Limelight limeLight;
    //    private boolean xJustPressed = false;
//    private boolean xHolding = false;
//    private boolean yJustPressed = false;
//    private boolean yHolding = false;
    private double x, y, rx;


    @Override
    public void runOpMode() {

        limeLight = new Limelight(hardwareMap);
        limeLight.initRedPipeline();
        limeLight.startDetect();


        waitForStart();
        while (opModeIsActive()){
            limeLight.periodic();
            telemetry.addData("Apriltag dist", limeLight.getDis());
            telemetry.addData("Apriltag X", limeLight.getX());
            telemetry.addData("Apriltag(PoI) Tx", limeLight.getTx());
            telemetry.addData("Apriltag ID", limeLight.getAprilTagID());
            telemetry.addData("Pitch", limeLight.getPitch());

            // shooter
            // telemetry
//            telemetry.addData("Shooter Target RPM", shooter.getTargetRPM());
//            telemetry.addData("Shooter Current RPM", shooter.getFlyWheelRPM());
//            telemetry.addData("PIDoutput", shooter.getCurrentPIDOutput());
//            telemetry.addData("Shooter At Target", shooter.isAtTargetRPM() ? "YES" : "NO");
        }

    }
}