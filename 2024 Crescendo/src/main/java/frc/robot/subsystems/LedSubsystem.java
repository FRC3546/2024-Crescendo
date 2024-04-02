package frc.robot.subsystems;

//wpilib
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.StrobeAnimation;

public class LedSubsystem extends SubsystemBase{
    
    public Color green = new Color(0,255,0);
    public Color white = new Color(255,255,255);

    private CANdle leds;
    private int startIndex;
    private int segmentSize;
    
    public LedSubsystem(int startIndex, int segmentSize){
        
        this.startIndex = startIndex;
        this.segmentSize = segmentSize;
        leds = new CANdle(9);
        CANdleConfiguration ledconfig = new CANdleConfiguration();
        ledconfig.stripType = LEDStripType.RGB;
        ledconfig.brightnessScalar = 0.5;
        leds.configAllSettings(ledconfig);
    }

    public void setAnimation(Animation animation) {
            leds.animate(animation);
    }

    public void clearAnimation(){
        leds.clearAnimation(0);
    }

    public void gold(){
        leds.setLEDs(255, 215, 0);
    }
    
     public void red(){
        leds.setLEDs(255, 0, 0);
    }

    public void green(){
        leds.setLEDs(green.red, green.green, green.blue);
    }

    public void blue(){
        leds.setLEDs(0, 0, 255);
    }

    public void orange(){
        leds.setLEDs(255, 136, 0);
    }
    
    public void off(){
        leds.setLEDs(0, 0, 0);
    }

    public void setStrobeAnimation(Color color, double speed){
        setAnimation(new StrobeAnimation(color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
    }

    public static class Color {
        public int red;
        public int green;
        public int blue;

        public Color(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }
    }


}
    