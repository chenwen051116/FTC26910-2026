package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class TestRedAuto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain MovetoShoot0;
        public PathChain Take1;
        public PathChain MovetoShoot1;
        public PathChain Take2;
        public PathChain MovetoShoot2;
        public PathChain Take3;
        public PathChain MovetoShoot3;
        public PathChain MoveoutAuto;

        public Paths(Follower follower) {
            MovetoShoot0 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(116.730, 130.697), new Pose(93.949, 113.238))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(37))
                    .build();

            Take1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(93.949, 113.238),
                                    new Pose(93.284, 80.647),
                                    new Pose(130.531, 83.806)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            MovetoShoot1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(130.531, 83.806), new Pose(93.783, 113.570))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37))
                    .build();

            Take2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(93.783, 113.570),
                                    new Pose(79.982, 55.206),
                                    new Pose(130.032, 58.864)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            MovetoShoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(130.032, 58.864),
                                    new Pose(94.282, 55.538),
                                    new Pose(93.949, 113.404)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37))
                    .build();

            Take3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(93.949, 113.404),
                                    new Pose(80.647, 31.926),
                                    new Pose(128.702, 34.919)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            MovetoShoot3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(128.702, 34.919),
                                    new Pose(97.109, 72.499),
                                    new Pose(93.949, 113.404)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37))
                    .build();

            MoveoutAuto = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(93.949, 113.404), new Pose(112.573, 92.785))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(37))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}