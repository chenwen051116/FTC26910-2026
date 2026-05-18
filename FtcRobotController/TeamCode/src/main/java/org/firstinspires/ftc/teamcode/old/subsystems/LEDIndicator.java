package org.firstinspires.ftc.teamcode.old.subsystems;

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
    public Color currentBallIndicatingColor = Color.Off;
    public Color currentShooterStatusIndicatingColor = Color.Off;
    public LEDShooterStatus currentShooterStatus = LEDShooterStatus.Off;


    public LEDIndicator(HardwareMap hardwareMap){
        shooterStatusIndicatingLED = hardwareMap.get(Servo.class, "led1");
        ballIndicatingLED1 = hardwareMap.get(Servo.class, "led2");
        ballIndicatingLED2 = hardwareMap.get(Servo.class, "led3");
    }

    public enum Color{
        Off(0),
        Red(0.288),
        Orange(0.333),
        Yellow(0.388),
        Sage(0.444),
        Green(0.500),
        Azure(0.555),
        Blue(0.611),
        Indigo(0.666),
        Violet(0.722),
        White(1.0);
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
        Idle(Color.Blue),
        Off(Color.White),
        Shooting(Color.Violet);
        private final Color color;
        LEDShooterStatus(Color color){
            this.color = color;
        }
    }

    public void setColorByBallCount(){
        switch (currentBallCount){
            case 0:
                currentBallIndicatingColor = Color.Red;
                break;
            case 1:
                currentBallIndicatingColor = Color.Orange;
                break;

            case 2:
                currentBallIndicatingColor = Color.Yellow;
                break;

            case 3:
                currentBallIndicatingColor = Color.Sage;
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
            currentBallIndicatingColor = Color.Green;
        } else{
            currentBallIndicatingColor = Color.Red;
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
