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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // 更改为 LinearOpMode

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name = "Pedro Pathing Linear Autonomous", group = "Autonomous")
@Configurable
public class TestRedAuto extends LinearOpMode { // 类名和继承更改

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    // 硬件定义（如果你的操作需要）
    private Shooter shooter;
    private Intake intake;
    private Limelight limelight;
    private Turret turret;

    public static int shootingTime = 1500;

    public static int deg = 180;


    @Override
    public void runOpMode() throws InterruptedException { // 核心方法
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // 1. 初始化
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));
        paths = new Paths(follower);
        // robot = new MyRobotHardware(hardwareMap); // 假设你有一个硬件类
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        limelight = new Limelight(hardwareMap);

        shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
        intake.setIntakeState(Intake.IntakeStates.Stop);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);


        // 2. 等待 Start 信号
        waitForStart();

        if (isStopRequested()) return;

        // --- 3. 路径执行序列（顺序执行）---

        // MovetoShoot0
        panelsTelemetry.debug("Current Task", "Starting MovetoShoot0");
        follower.followPath(paths.MovetoShoot0);
        waitForPath();

        // Shooting 0
        panelsTelemetry.debug("Current Task", "Executing Shooting 0");
        startShooting();
        sleep(shootingTime);
        stopShooting();


        // Take 1
        panelsTelemetry.debug("Current Task", "Starting Take1");
        follower.followPath(paths.Take1);
        startIntake();
        waitForPath();
        stopIntake();


        // MovetoShoot1
        panelsTelemetry.debug("Current Task", "Starting MovetoShoot1");
        follower.followPath(paths.MovetoShoot1);
        waitForPath();

        // Shooting 1
        panelsTelemetry.debug("Current Task", "Executing Shooting 1");
        startShooting();
        sleep(shootingTime);
        stopShooting();


        // Take2
        panelsTelemetry.debug("Current Task", "Starting Take2");
        follower.followPath(paths.Take2);

        startIntake();
        waitForPath();
        stopIntake();

        // MovetoShoot2
        panelsTelemetry.debug("Current Task", "Starting MovetoShoot2");
        follower.followPath(paths.MovetoShoot2);
        waitForPath();

        // Shooting 2
        panelsTelemetry.debug("Current Task", "Executing Shooting 2");
        startShooting();
        sleep(shootingTime);
        stopShooting();

        // Take3
        panelsTelemetry.debug("Current Task", "Starting Take3");
        follower.followPath(paths.Take3);

        startIntake();
        waitForPath();
        stopIntake();

        // MovetoShoot3
        panelsTelemetry.debug("Current Task", "Starting MovetoShoot3");
        follower.followPath(paths.MovetoShoot3);
        waitForPath();

        // Shooting 3
        panelsTelemetry.debug("Current Task", "Executing Shooting3");
        startShooting();
        sleep(shootingTime);
        stopShooting();

        // Moveout
        panelsTelemetry.debug("Current Task", "Starting MoveoutAuto");
        follower.followPath(paths.MoveoutAuto);
        waitForPath();



    }

    // 辅助方法：将遥测记录移到单独的方法中
    private void logTelemetry() {
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public void periodics(){
        turret.periodic();
        shooter.periodic();
        intake.periodic();
    }

    public void startShooting(){
        turret.tx = limelight.getTx();
        shooter.setShooterStatus(Shooter.ShooterStatus.Shooting);
        turret.updateAutoShoot(true);
    }

    public void stopShooting(){
        turret.updateAutoShoot(false);
        shooter.setShooterStatus(Shooter.ShooterStatus.Idling);
    }

    public void startIntake(){
        intake.setIntakeState(Intake.IntakeStates.Ball_In);
    }

    public void stopIntake(){
        intake.setIntakeState(Intake.IntakeStates.Stop);
    }

    public void waitForPath(){
        while (opModeIsActive() && follower.isBusy()) {
            periodics();
            follower.update();
            logTelemetry(); // 持续记录数据
        }
    }

    // Paths 类的定义保持不变
    public static class Paths {
        // ... (你的路径定义) ...

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
                    .setLinearHeadingInterpolation(Math.toRadians(deg + 37), Math.toRadians(deg + 37))
                    .setGlobalDeceleration(0.95)
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
                    .setLinearHeadingInterpolation(Math.toRadians(deg + 0), Math.toRadians(deg + 37))
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
                                    new Pose(128.370, 58.864),
                                    new Pose(106.420, 80.647),
                                    new Pose(130.199, 56.203),
                                    new Pose(143.501, 88.296),
                                    new Pose(93.949, 113.404)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(deg - 0), Math.toRadians(deg - 37))
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
                    .setLinearHeadingInterpolation(Math.toRadians(deg - 0), Math.toRadians(deg - 37))
                    .build();

            MoveoutAuto = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(93.949, 113.404), new Pose(112.573, 92.785))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(deg - 37), Math.toRadians(deg - 37))
                    .build();
        }
    }
}