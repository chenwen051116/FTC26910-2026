package org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

@Config
public class PIDControllerFactory {
    @Config
    public static class TurretPIDController extends PIDController {
        public static double kp = -0.0003, ki = 0, kd = -0.00001;
        public static double kf = 0;
        public static double tolerance = 0.01;

        private TurretPIDController() {
            super(kp, ki, kd);
            setTolerance(tolerance);
            setSetPoint(0);
        }

        public double calculatePower(double currentPosition, double targetPosition) {
            setSetPoint(targetPosition);
            setTolerance(tolerance);
            setPIDF(kp, ki, kd, kf);
            return Math.max(-0.5, Math.min(0.5, calculate(currentPosition)));
        }
    }

    @Config
    public static class FlywheelPIDController extends PIDController {
        // PID FOR V = 12.5
        public static double kp = -0.24, ki = 0, kd = -0.003;
        public static double ks = 0;
        public static double kv = 0.000245;
        public static double threshold = 300, tolerance = 0;

        // 13.5V : KS = 0.165, vf = 0.000195

        private FlywheelPIDController() {
            super(kp, ki, kd);
            setTolerance(tolerance);
        }

        public double calculatePower(double currentRPM, double targetRPM) {
            if (targetRPM == 0) {
                return 0;
            }

            double rpmDifference = targetRPM - currentRPM;
            setPID(kp, ki, kd);
            setTolerance(tolerance);
            if (rpmDifference > threshold) {
                return 1;
            } else if (rpmDifference < -threshold) {
                return 0;
            }
            double pidOutput = calculate(rpmDifference / 100) + kv * targetRPM + ks;
            return Math.max(-1, Math.min(1, pidOutput));
        }
    }

    public static TurretPIDController createTurretPIDController() {
        return new TurretPIDController();
    }

    public static FlywheelPIDController createFlywheelPIDController() {
        return new FlywheelPIDController();
    }
}