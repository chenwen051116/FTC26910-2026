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

    public int currentBallCount = 0;
    public Color currentColor = Color.Off;



    public LEDIndicator(HardwareMap hardwareMap){
        firstLED = hardwareMap.get(Servo.class, "led1");
        secondLED = hardwareMap.get(Servo.class, "led2");
        thirdLED = hardwareMap.get(Servo.class, "led3");
    }

    public enum Color{
        Off(0),
        Red(0.277),
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

    public void setColorByBallCount(int ballCount){
        switch (ballCount){
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
                currentColor = Color.Green;
                break;
        }
    }

    @Override
    public void periodic(){
        updateLEDColor();
    }
}
