package frc.robot.subsystems;

//wpilib
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//vendordeps
import com.ctre.phoenix.CANifier;

public class LedSubsystem extends SubsystemBase{

    public CANifier led = new CANifier(9);

    public void LEDSet(double r1, double g1, double b1){
        led.setLEDOutput(b1, CANifier.LEDChannel.LEDChannelA); // Blue
        led.setLEDOutput(r1, CANifier.LEDChannel.LEDChannelB); // Red
        led.setLEDOutput(g1, CANifier.LEDChannel.LEDChannelC); // Green
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
}
    