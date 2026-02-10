package org.firstinspires.ftc.teamcode.subSystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class LEDIndicator extends SubsystemBase {
    private final Servo shooterStatusIndicatingLED;
    private final Servo ballIndicatingLED1;
    private final Servo ballIndicatingLED2;


    public boolean shootingStatus = false;
    public boolean onTarget = false;

    public int currentBallCount = 0;
    public Color currentBallIndicatingColor = Color.OFF;
    public Color currentShooterStatusIndicatingColor = Color.OFF;
    public LEDShooterStatus currentShooterStatus = LEDShooterStatus.OFF;


    public LEDIndicator(HardwareMap hardwareMap){
        shooterStatusIndicatingLED = hardwareMap.get(Servo.class, "led1");
        ballIndicatingLED1 = hardwareMap.get(Servo.class, "led2");
        ballIndicatingLED2 = hardwareMap.get(Servo.class, "led3");
    }

    public enum Color{
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
        private final double colorPWM;
        Color(double colorPWM){
            this.colorPWM = colorPWM;
        }
    }

    public void setLEDColor(Color targetColor){
        currentBallIndicatingColor = targetColor;
    }

    public void updateBallIndicatingLEDColor(){
        ballIndicatingLED1.setPosition(currentBallIndicatingColor.colorPWM);
        ballIndicatingLED2.setPosition(currentBallIndicatingColor.colorPWM);
    }

    public void updateShooterStatusIndicatingLEDColor(){
        shooterStatusIndicatingLED.setPosition(currentShooterStatusIndicatingColor.colorPWM);
    }

    public void updateBallCount(int ballCount){
        currentBallCount = ballCount;
    }

    public enum LEDShooterStatus{
        IDLE(Color.BLUE),
        OFF(Color.WHITE),
        SHOOTING(Color.VIOLET);
        private final Color color;
        LEDShooterStatus(Color color){
            this.color = color;
        }
    }

    public void setColorByBallCount(){
        switch (currentBallCount){
            case 0:
                currentBallIndicatingColor = Color.RED;
                break;
            case 1:
                currentBallIndicatingColor = Color.ORANGE;
                break;

            case 2:
                currentBallIndicatingColor = Color.YELLOW;
                break;

            case 3:
                currentBallIndicatingColor = Color.SAGE;
                break;
        }
    }



    public void updateColorByShooterStatus(){
        currentShooterStatusIndicatingColor = currentShooterStatus.color;
    }

    public void setShooterStatusTo(LEDShooterStatus targetShooterStatus){
        currentShooterStatus = targetShooterStatus;

    }
    public void setShooterStatusTo(boolean targetShootingState){
        shootingStatus = targetShootingState;
    }


    public void setOnTarget(boolean targetOnTargetState){
        onTarget = !targetOnTargetState;
    }

    public void setColorByOnTargetState(){
        if (onTarget){
            currentBallIndicatingColor = Color.GREEN;
        } else{
            currentBallIndicatingColor = Color.RED;
        }
    }

    @Override
    public void periodic(){
        if (shootingStatus){
            setColorByOnTargetState();
        } else{
            setColorByBallCount();
        }
        updateBallIndicatingLEDColor();
        updateColorByShooterStatus();
        updateShooterStatusIndicatingLEDColor();
    }
}
