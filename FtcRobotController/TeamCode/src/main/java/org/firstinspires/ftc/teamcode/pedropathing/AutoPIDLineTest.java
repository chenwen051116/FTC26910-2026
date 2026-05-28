package org.firstinspires.ftc.teamcode.pedropathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.Drawing;
import org.firstinspires.ftc.teamcode.hardware.HardwareCommandCache;

import java.util.List;

@Config
@TeleOp(name = "Auto PID Line Test", group = "Pedro Pathing")
public class AutoPIDLineTest extends OpMode {
    public static double startX = 72;
    public static double startY = 72;
    public static double heading = 0;
    public static double distance = 40;

    private Follower follower;
    private Path forwards;
    private Path backwards;
    private List<LynxModule> allHubs;
    private boolean forward = true;

    @Override
    public void init() {
        Drawing.init();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        HardwareCommandCache.resetCommandCache();
        follower = Constants.createFollower(hardwareMap);
        allHubs = HardwareCommandCache.enableManualBulkCaching(hardwareMap);
        follower.setStartingPose(startPose());
    }

    @Override
    public void init_loop() {
        HardwareCommandCache.clearBulkCache(allHubs);
        follower.update();
        Drawing.drawDebug(follower);
        addTelemetry();
    }

    @Override
    public void start() {
        follower.activateAllPIDFs();
        buildPaths();
        forward = true;
        follower.followPath(forwards);
    }

    @Override
    public void loop() {
        HardwareCommandCache.clearBulkCache(allHubs);
        follower.update();
        Drawing.drawDebug(follower);

        if (!follower.isBusy()) {
            forward = !forward;
            follower.followPath(forward ? forwards : backwards);
        }

        addTelemetry();
    }

    @Override
    public void stop() {
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0, 0, 0, true);
    }

    private void buildPaths() {
        Pose start = startPose();
        Pose end = new Pose(startX + distance, startY, heading);

        forwards = buildLinePath(start, end);
        backwards = buildLinePath(end, start);
    }

    private Path buildLinePath(Pose start, Pose end) {
        Path path = new Path(new BezierLine(start, end));
        path.setConstantHeadingInterpolation(heading);
        return path;
    }

    private Pose startPose() {
        return new Pose(startX, startY, heading);
    }

    private void addTelemetry() {
        Pose pose = follower.getPose();
        telemetry.addData("Matches Pedro Line Test path setup", true);
        telemetry.addData("Driving forward", forward);
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading", pose.getHeading());
        telemetry.addData("T value", follower.getCurrentTValue());
        telemetry.update();
    }
}
