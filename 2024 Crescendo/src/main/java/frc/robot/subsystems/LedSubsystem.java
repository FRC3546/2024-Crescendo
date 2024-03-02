package frc.robot.subsystems;

//wpilib
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

//vendordeps
 

public class LedSubsystem extends SubsystemBase{
    
    private CANdle leds;
    
    public LedSubsystem(){
        
        leds = new CANdle(9);
        CANdleConfiguration ledconfig = new CANdleConfiguration();
        ledconfig.stripType = LEDStripType.RGB;
        ledconfig.brightnessScalar = 0.5;
        leds.configAllSettings(ledconfig);
        

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


}
    