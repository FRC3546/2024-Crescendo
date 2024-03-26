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
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class IntakeSubsystem extends SubsystemBase{
    //motor
    private VictorSPX intakeMotor;
    //sensor
    DigitalInput noteSensor;

    public IntakeSubsystem(){
        noteSensor = new DigitalInput(1);
        intakeMotor = new VictorSPX(13);
        intakeMotor.setInverted(true);
        intakeMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void intake(double speed){
        intakeMotor.set(VictorSPXControlMode.PercentOutput, speed);
    }

    public void stopIntake(){
        intakeMotor.set(VictorSPXControlMode.Disabled, 0);
    }

    public double getIntakeSpeed(){
        return intakeMotor.getMotorOutputPercent();
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