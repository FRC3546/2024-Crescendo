package frc.robot.subsystems;

//wpilib
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;

//vendordeps
 

public class LedSubsystem extends SubsystemBase{
    
    private CANdle leds;
    RainbowAnimation shootAnimation = new RainbowAnimation(0.5, 0.5, 8);
    
    public LedSubsystem(){
        
        leds = new CANdle(9);
        CANdleConfiguration ledconfig = new CANdleConfiguration();
        ledconfig.stripType = LEDStripType.RGB;
        ledconfig.brightnessScalar = 0.5;
        leds.configAllSettings(ledconfig);
       
        

    }

    public void gold(){
        leds.setLEDs(255, 215, 0);
    }
    
     public void red(){
        leds.setLEDs(255, 0, 0);
    }

    public void green(){
        leds.setLEDs(0, 255, 0);
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

    public void rainbow(){
        leds.animate(shootAnimation);
    }


}
    