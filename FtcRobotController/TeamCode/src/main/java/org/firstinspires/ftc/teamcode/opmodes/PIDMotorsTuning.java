package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;

/**
 * Clean PID Tuning OpMode for Shooter
 * Dashboard-only: Manual parameter entry + Real-time graphing
 * No gamepad controls - just pure PID tuning
 */
@Config
@TeleOp(name = "Shooter PID Tuning", group = "Tuning")
public class PIDMotorsTuning extends OpMode {
    
    // Dashboard tunable parameters - automatically detected by @Config on class
    public static double Kp = 0.001;
    
    public static double Ki = 0.0001;
    
    public static double Kd = 0.0;
    
    public static double pidThreshold = 200.0;
    
    public static double tolerance = 50.0;
    
    public static double targetRPM = 2000.0;
    
    private Shooter shooter;
    private Telemetry dashboardTelemetry;
    private FtcDashboard dashboard;
    
    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        
        // Update PID parameters
        Shooter.Kp = Kp;
        Shooter.Ki = Ki;
        Shooter.Kd = Kd;
        Shooter.PIDThreshold = pidThreshold;
        Shooter.tolerance = tolerance;
        
        dashboardTelemetry.addData("Status", "Clean PID Tuning Ready - Use Dashboard to adjust parameters");
        dashboardTelemetry.update();
    }
    
    @Override
    public void loop() {
        // Update PID parameters from dashboard
        Shooter.Kp = Kp;
        Shooter.Ki = Ki;
        Shooter.Kd = Kd;
        Shooter.PIDThreshold = pidThreshold;
        Shooter.tolerance = tolerance;
        
        // Set target and update PID
        shooter.setTargetRPMTo(targetRPM);
        shooter.updateShooterPID();
        
        // Create telemetry packet for graphing
        TelemetryPacket packet = new TelemetryPacket();
        
        // Add data for graphing
        packet.put("Target RPM", targetRPM);
        packet.put("Current RPM", shooter.getFlyWheelRPM());
        packet.put("RPM Error", targetRPM - shooter.getFlyWheelRPM());
        packet.put("Motor Power", shooter.getCurrentMotorPower());
        packet.put("PID Output", shooter.getCurrentMotorPIDOutput());
        
        // Add PID parameters
        packet.put("Kp", Kp);
        packet.put("Ki", Ki);
        packet.put("Kd", Kd);
        packet.put("Threshold", pidThreshold);
        packet.put("Tolerance", tolerance);
        packet.put("At Target", shooter.isAtTargetRPM() ? "YES" : "NO");
        
        // Send packet to dashboard for graphing
        dashboard.sendTelemetryPacket(packet);
        
        // Also update regular telemetry (minimal, clean display)
        dashboardTelemetry.addData("Target RPM", "%.1f", targetRPM);
        dashboardTelemetry.addData("Current RPM", "%.1f", shooter.getFlyWheelRPM());
        dashboardTelemetry.addData("RPM Error", "%.1f", targetRPM - shooter.getFlyWheelRPM());
        dashboardTelemetry.addData("Motor Power", "%.3f", shooter.getCurrentMotorPower());
        dashboardTelemetry.addData("PID Output", "%.3f", shooter.getCurrentMotorPIDOutput());
        dashboardTelemetry.addData("At Target", shooter.isAtTargetRPM() ? "YES" : "NO");
        dashboardTelemetry.addData("", "");
        dashboardTelemetry.addData("Kp", "%.6f", Kp);
        dashboardTelemetry.addData("Ki", "%.6f", Ki);
        dashboardTelemetry.addData("Kd", "%.6f", Kd);
        dashboardTelemetry.addData("Threshold", "%.1f", pidThreshold);
        dashboardTelemetry.addData("Tolerance", "%.1f", tolerance);
        dashboardTelemetry.update();
    }
    
    @Override
    public void stop() {
        shooter.setCompleteStop();
        dashboardTelemetry.addData("Status", "Stopped");
        dashboardTelemetry.update();
    }
}
