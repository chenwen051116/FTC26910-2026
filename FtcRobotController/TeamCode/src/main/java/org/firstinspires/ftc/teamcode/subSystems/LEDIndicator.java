package org.firstinspires.ftc.teamcode.subSystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class LEDIndicator extends SubsystemBase {
    private final Servo firstLED;
    private final Servo secondLED;
    private final Servo thirdLED;


    public boolean shootingState = false;
    public boolean onTarget = false;

    public int currentBallCount = 0;
    public Color currentColor = Color.Off;



    public LEDIndicator(HardwareMap hardwareMap){
        firstLED = hardwareMap.get(Servo.class, "led1");
        secondLED = hardwareMap.get(Servo.class, "led2");
        thirdLED = hardwareMap.get(Servo.class, "led3");
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
        currentColor = targetColor;
    }

    public void updateLEDColor(){
        firstLED.setPosition(currentColor.colorPWM);
        secondLED.setPosition(currentColor.colorPWM);
        thirdLED.setPosition(currentColor.colorPWM);
    }

    public void updateBallCount(int ballCount){
        currentBallCount = ballCount;
    }

    public void setColorByBallCount(){
        switch (currentBallCount){
            case 0:
                currentColor = Color.Red;
                break;
            case 1:
                currentColor = Color.Orange;
                break;

            case 2:
                currentColor = Color.Yellow;
                break;

            case 3:
                currentColor = Color.Sage;
                break;
        }
    }

    public void setShootingState(boolean targetShootingState){
        shootingState = targetShootingState;
    }


    public void setOnTarget(boolean targetOnTargetState){
        onTarget = !targetOnTargetState;
    }

    public void setColorByOnTargetState(){
        if (onTarget){
            currentColor = Color.Green;
        } else{
            currentColor = Color.Red;
        }
    }

    @Override
    public void periodic(){
        if (shootingState){
            setColorByOnTargetState();
        } else{
            setColorByBallCount();
        }
        updateLEDColor();
    }
}
