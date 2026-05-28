package org.firstinspires.ftc.teamcode.subsystems.led;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.HardwareCommandCache;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter.ShooterState;

public class LEDSet extends SubsystemBase {
    private enum Color {
        OFF(0),
        RED(0.288),
        ORANGE(0.333),
        YELLOW(0.388),
        SAGE(0.444),
        GREEN(0.500),
        AZURE(0.555),
        BLUE(0.611),
        INDIGO(0.666),
        VIOLET(0.722),
        WHITE(1.0);

        private final double pwm;

        Color(double pwm) {
            this.pwm = pwm;
        }
    }

    private final Color[] colorByBallCount = new Color[]{
            Color.OFF, Color.RED, Color.YELLOW, Color.GREEN
    };

    private final Servo ballIndicator1;
    private final Servo ballIndicator2;
    private final Servo shooterIndicator;
    private int ballCount;
    private ShooterState shooterState = ShooterState.OFF;

    public LEDSet(Servo ballIndicator1, Servo ballIndicator2, Servo shooterIndicator) {
        this.ballIndicator1 = ballIndicator1;
        this.ballIndicator2 = ballIndicator2;
        this.shooterIndicator = shooterIndicator;
        ballCount = 0;
    }

    public void setBallCount(int ballCount) {
        this.ballCount = ballCount;
    }

    public void setShooterState(ShooterState shooterState) {
        this.shooterState = shooterState;
    }

    @Override
    public void periodic() {
        HardwareCommandCache.setServoPosition(ballIndicator1, colorByBallCount[ballCount].pwm);
        HardwareCommandCache.setServoPosition(ballIndicator2, colorByBallCount[ballCount].pwm);

        Color shooterStateDisplayColor = Color.OFF;
        switch (shooterState) {
            case SHOOTING:
                shooterStateDisplayColor = Color.INDIGO;
                break;
            case IDLE:
                shooterStateDisplayColor = Color.BLUE;
                break;
        }
        HardwareCommandCache.setServoPosition(shooterIndicator, shooterStateDisplayColor.pwm);
    }
}
