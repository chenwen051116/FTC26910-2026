package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

@Config
public class PIDControllerFactory {
    @Config
    public static class FlywheelPIDController extends PIDController {
        // PID FOR V = 12.5
        public static double kp = -0.24, ki = 0, kd = -0.003;
        public static double ks = 0;
        public static double kv = 0.00029;
        public static double threshold = 200, tolerance = 0;


        // 13.5V : KS = 0.165, vf = 0.000195

        private FlywheelPIDController() {
            super(kp, ki, kd);
            setTolerance(tolerance);
        }

        public static void addFlywheelKv(double increment) {
            kv += increment;
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

    public static FlywheelPIDController createFlywheelPIDController() {
        return new FlywheelPIDController();
    }
}
