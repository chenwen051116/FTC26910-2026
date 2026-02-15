//package org.firstinspires.ftc.teamcode.tuning;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.subsystems.Shooter;
//
//@Config
//@TeleOp(name = "PID Flywheel Tuning")
//public final class PIDveltest extends CommandOpMode {
//    // Tunable parameters - can be adjusted via FTC Dashboard-only
//    public static double targetRPM = 1000;
//
//    private Shooter shooter;
//    @Override
//    public void initialize() {
//        // Setup telemetry for dashboard
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        // Initialize shooter subsystem
//        shooter = new Shooter(hardwareMap);
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.addData("Instructions", "Use FTC Dashboard to tune PID parameters");
//        telemetry.update();
//    }
//
//
//    @Override
//    public void run() {
//        // Update PID parameters from dashboard
//        Shooter.Kp = kp;
//        Shooter.Ki = ki;
//        Shooter.Kd = kd;
//
//        // Set target RPM
//        shooter.setTargetRPM(targetRPM);
//
//        // Update PID controller
//        shooter.updateFlywheelPID();
//
//        // Display telemetry
//        shooter.updateTelemetry();
//        telemetry.addData("Target RPM (Dashboard)", targetRPM);
//        telemetry.addData("Tolerance", tolerance);
//        telemetry.addData("Error", targetRPM - shooter.getFlyWheelRPM());
//        telemetry.addData("PIDoutput", shooter.PIDoutput);
//
//
//        // Control instructions
//        telemetry.addLine();
//        telemetry.addData("Controls", "Adjust PID values via FTC Dashboard");
//        telemetry.addData("Start with Kp", "Increase Kp until oscillation occurs");
//        telemetry.addData("Then add Ki", "Small values like 0.0001");
//        telemetry.addData("Finally Kd", "Usually 0 for velocity control");
//
//        telemetry.update();
//    }
//}