package org.firstinspires.ftc.teamcode.old.subsystems;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

public class Limelight extends SubsystemBase {
    // Hardware (motor servo...)
    private final Limelight3A limelight;
    private LLResult aprilTagLatestResult;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean llenable = false;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // fast updates

    }



    public void initBluePipeline(){
        limelight.pipelineSwitch(1);
        limelight.start();
    }
    public void initRedPipeline(){
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    // pattern:
    // 23 = PPG
    // 22 = PGP
    // 21 = GPP

    public void initPatternPipeline(){
        limelight.pipelineSwitch(9);
        limelight.start();
    }
    public double getPitch() {
        if (llenable && hasTarget()) {
            return aprilTagLatestResult.getFiducialResults().get(0)
                    .getTargetPoseCameraSpace().getOrientation().getPitch(AngleUnit.DEGREES);
        }
        return 0;
    }
    public double getX() {
        if (llenable && hasTarget()) {
            return aprilTagLatestResult.getFiducialResults().get(0)
                    .getTargetPoseCameraSpace().getPosition().x;
        }
        return 0;
    }
    public double getY() {
        if (llenable && hasTarget()) {
            return aprilTagLatestResult.getFiducialResults().get(0)
                    .getTargetPoseCameraSpace().getPosition().y;
        }
        return 0;
    }
    public double getZ() {
        if (llenable && hasTarget()) {
            return aprilTagLatestResult.getFiducialResults().get(0)
                    .getTargetPoseCameraSpace().getPosition().z;
        }
        return 0;
    }
    public double getTx() {
        if (llenable && hasTarget()) {
            return aprilTagLatestResult.getTx();
        }
        return 0;
    }
    // NOTE for AprilTag: size is 16.5cm * 16.5cm

    public boolean onTarget(){
        return -1 < abs(getTx()) && abs(getTx()) < 1;
    }
    public double getDis() {
        // Fetch most recent vision result each scheduler loop
        if (llenable && hasTarget()){
            List<LLResultTypes.FiducialResult> fiducialResults = aprilTagLatestResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                // this will return the distance by pythag. The relative x, y, z position
                // of the target to the robot is first aquired, then calculated
                    return sqrt(fr.getTargetPoseCameraSpace().getPosition().y * fr.getTargetPoseCameraSpace().getPosition().y
                            + (fr.getTargetPoseCameraSpace().getPosition().x) * (fr.getTargetPoseCameraSpace().getPosition().x)
                            + (fr.getTargetPoseCameraSpace().getPosition().z) * (fr.getTargetPoseCameraSpace().getPosition().z)
                    );
            }
        }
        return 0;
    }

    public void startDetect(){
        llenable = true;
    }
    public void stopDetect(){
        llenable = false;
    }
    public boolean hasTarget() { //Has to be a METHOD instead a VARIABLE since limelight is constantly updating
        return aprilTagLatestResult != null && aprilTagLatestResult.isValid(); //This will return true only if the data is not empty and valid
    }

    public LLResult getAprilTagResult() {
        return aprilTagLatestResult;
    }
    public int getAprilTagID() {
        return llenable && hasTarget() && !aprilTagLatestResult.getFiducialResults().isEmpty() ?
                aprilTagLatestResult.getFiducialResults().get(0).getFiducialId() : -1;
    }
    @Override
    public void periodic() {
        // This is called automatically by FTCLib’s scheduler every cycle
        if (llenable && limelight.getLatestResult() != null) {
            aprilTagLatestResult = limelight.getLatestResult();
        }
    }
}


