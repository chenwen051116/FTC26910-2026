package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.shooter.Flywheel.FlywheelState;

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

    private Servo ballIndicator1;
    private Servo ballIndicator2;
    private Servo flywheelStateIndicator;
    private int ballCount;
    private FlywheelState flywheelState = FlywheelState.OFF;

    public LEDSet(HardwareMap hardwareMap) {
        ballIndicator1 = hardwareMap.get(Servo.class, "ball_indicator_1");
        ballIndicator2 = hardwareMap.get(Servo.class, "ball_indicator_2");
        flywheelStateIndicator = hardwareMap.get(Servo.class, "flywheel_state_indicator");
        ballCount = 0;
    }

    public void setBallCount(int ballCount) {
        this.ballCount = ballCount;
    }

    public void setFlywheelState(FlywheelState flywheelState) {
        this.flywheelState = flywheelState;
    }

    @Override
    public void periodic() {
        ballIndicator1.setPosition(colorByBallCount[ballCount].pwm);
        ballIndicator2.setPosition(colorByBallCount[ballCount].pwm);

        Color flywheelStateDisplayColor = Color.OFF;
        switch (flywheelState) {
            case SHOOTING:
                flywheelStateDisplayColor = Color.INDIGO;
                break;
            case IDLE:
                flywheelStateDisplayColor = Color.BLUE;
                break;
        }
        flywheelStateIndicator.setPosition(flywheelStateDisplayColor.pwm);
    }
}
