package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.revrobotics.SparkAbsoluteEncoder;

public class ArmSubsystem extends SubsystemBase{


    private CANSparkMax rotateMotor1;
    private CANSparkMax rotateMotor2;
    private SparkAbsoluteEncoder throughBoreEncoder;
    private DoubleSolenoid extendSolenoid;


    public ArmSubsystem(){

        rotateMotor1 = new CANSparkMax (0, MotorType.kBrushless);
        rotateMotor2 = new CANSparkMax (1, MotorType.kBrushless);
        // throughBoreEncoder = new SparkAbsoluteEncoder(rotateMotor1, )
        //create CANSparkMax IDs
        extendSolenoid = new DoubleSolenoid (PneumaticsModuleType.CTREPCM, 0, 1);

        rotateMotor2.follow(rotateMotor1);
        

    } 

    
    
    public void rotateArmUp(){
        
        rotateMotor1.set(1);
        // rotateMotor2.set(1);
        
    }
    public void rotateArmDown(){
        rotateMotor1.set(-1);
        // rotateMotor2.set(-1);
    }
    public void StopArm(){
        rotateMotor1.stopMotor();
        // rotateMotor2.set(0);
    }


    public void extendArm(){

        extendSolenoid.set(Value.kForward);
    }

    public void retractArm(){

        extendSolenoid.set(Value.kReverse);
    }    

    public void toggleArm(){
         
        extendSolenoid.toggle();
    }
} 







