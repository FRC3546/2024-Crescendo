package frc.robot.subsystems;

//wpilib
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//vendordeps
import com.ctre.phoenix.CANifier;

public class LedSubsystem extends SubsystemBase{

    public CANifier led = new CANifier(9);

    //enter value RGB values from 0-255
    public void LEDSet(int red, int green, int blue){
        led.setLEDOutput(blue/255, CANifier.LEDChannel.LEDChannelA); // Blue
        led.setLEDOutput(red/255, CANifier.LEDChannel.LEDChannelB); // Red
        led.setLEDOutput(green/255, CANifier.LEDChannel.LEDChannelC); // Green
    }

    public void LEDBlue(){
        led.setLEDOutput(1, CANifier.LEDChannel.LEDChannelA); // Blue
        led.setLEDOutput(0, CANifier.LEDChannel.LEDChannelB); // Red
        led.setLEDOutput(0, CANifier.LEDChannel.LEDChannelC); // Green
    }

    public void LEDRed(){
        led.setLEDOutput(0, CANifier.LEDChannel.LEDChannelA); // Blue
        led.setLEDOutput(1, CANifier.LEDChannel.LEDChannelB); // Red
        led.setLEDOutput(0, CANifier.LEDChannel.LEDChannelC); // Green
    } 

    public void LEDOrange(){
        led.setLEDOutput(0, CANifier.LEDChannel.LEDChannelA); // Blue
        led.setLEDOutput(1, CANifier.LEDChannel.LEDChannelB); // Red
        led.setLEDOutput(0.5, CANifier.LEDChannel.LEDChannelC); // Green
    } 

    public void LEDOff(){
        led.setLEDOutput(0, CANifier.LEDChannel.LEDChannelA); // Blue
        led.setLEDOutput(0, CANifier.LEDChannel.LEDChannelB); // Red
        led.setLEDOutput(0, CANifier.LEDChannel.LEDChannelC); // Green
    }
    
    public void purple(){
        int red = 0;
        int blue = 0;
        int green = 0;
        LEDSet(blue, red, green);
    }


}
    