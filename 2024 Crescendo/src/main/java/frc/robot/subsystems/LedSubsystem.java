package frc.robot.subsystems;

//wpilib
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//vendordeps
import com.ctre.phoenix.CANifier;

public class LedSubsystem extends SubsystemBase{

    public CANifier led = new CANifier(9);

    //enter value RGB values from 0-255
    //converts the RGB values into percentages to given to the Canifier
    public void LEDSet(int red, int green, int blue){
        led.setLEDOutput(red/255, CANifier.LEDChannel.LEDChannelB); // Red
        led.setLEDOutput(green/255, CANifier.LEDChannel.LEDChannelC); // Green
        led.setLEDOutput(blue/255, CANifier.LEDChannel.LEDChannelA); // Blue
    }

    public void red(){
        int red = 255;
        int blue = 0;
        int green = 0;
        LEDSet(red, green, blue);
    }

    public void green(){
        int red = 0;
        int blue = 0;
        int green = 255;
        LEDSet(red, green, blue);
    }

    public void blue(){
        int red = 0;
        int blue = 255;
        int green = 0;
        LEDSet(red, green, blue);
    }

    public void orange(){
        int red = 255;
        int blue = 0;
        int green = 0;
        LEDSet(red, green, blue);
    }
    
    public void off(){
        int red = 0;
        int blue = 0;
        int green = 0;
        LEDSet(red, green, blue);
    }


}
    