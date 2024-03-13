package frc.robot.subsystems;

//robot
import frc.robot.Constants;

//wpilib
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//vendordeps
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

public class IntakeSubsystem extends SubsystemBase{
    //motor
    private VictorSP intakeMotor;
    //sensor
    DigitalInput noteSensor;

    public IntakeSubsystem(){
        noteSensor = new DigitalInput(1);
        intakeMotor = new VictorSP(0);
        intakeMotor.setInverted(true);
    }

    public void intake(double speed){
        intakeMotor.set(speed);
    }

    public void stopIntake(){
        intakeMotor.stopMotor();
    }

    public double getIntakeSpeed(){
        return intakeMotor.get();
    }

    public  boolean getSensorValue(){
        // the note sensor returns true by default, so the value needs to be 
        // flipped so it returns true when detecting a piece
        return !noteSensor.get();
    }

    @Override
    public void periodic(){
        Shuffleboard.update();
    }


}