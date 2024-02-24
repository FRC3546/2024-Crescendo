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

public class ShooterSubsystem extends SubsystemBase{

    //dashboard
    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    // private GenericEntry inputUpperShooterSpeed = tab.add("upper shooter speed", 0).getEntry();
    // private GenericEntry inputLowerShooterSpeed = tab.add("lower shooter speed", 0).getEntry();
    private GenericEntry inputShooterSpeed = tab.add("PLEASE WORK speed", 0).getEntry();

    //motors
    private CANSparkMax upperShooterMotor;
    private CANSparkMax lowerShooterMotor;

    //pid controllers
    private SparkPIDController lowerPIDController;
    private SparkPIDController upperPIDController;



    //not used but please leave
    private double upperShooterSpeed;
    private double lowerShooterSpeed;
    
    public ShooterSubsystem(){

        

        lowerShooterMotor = new CANSparkMax (31, MotorType.kBrushless);
        upperShooterMotor = new CANSparkMax (30, MotorType.kBrushless);

        upperShooterMotor.restoreFactoryDefaults();
        lowerShooterMotor.restoreFactoryDefaults();

        lowerPIDController = lowerShooterMotor.getPIDController();
        upperPIDController = upperShooterMotor.getPIDController();

        lowerPIDController.setP(0.003);
        lowerPIDController.setI(0);
        lowerPIDController.setD(0.055);
        lowerPIDController.setIZone(0);
        lowerPIDController.setFF(0);
        lowerPIDController.setOutputRange(0, 0.8);

        upperPIDController.setP(0.003);
        upperPIDController.setI(0);
        upperPIDController.setD(0.055);
        upperPIDController.setIZone(0);
        upperPIDController.setFF(0);
        upperPIDController.setOutputRange(0, 0.8);

        lowerShooterMotor.setSmartCurrentLimit(40);
        upperShooterMotor.setSmartCurrentLimit(40);
        
        lowerShooterMotor.burnFlash();
        upperShooterMotor.burnFlash();

        // Shuffleboard.getTab("Shooter").add("upper shooter RPM", getUpperShooterRPM());
        // Shuffleboard.getTab("Shooter").add("lower shooter RPM", getLowerShooterRPM());
        // Shuffleboard.getTab("Shooter").add("sensor value", noteSensor);
        // Shuffleboard.getTab("Shooter").add("note sensor boolean", getSensorValue());

    }

    

     public void runShooter(double upperShooterSpeed, double lowerShooterSpeed){
        
         this.upperShooterSpeed = upperShooterSpeed;
         this.lowerShooterSpeed = lowerShooterSpeed;

         upperShooterMotor.set(Math.abs(upperShooterSpeed));
         lowerShooterMotor.set(Math.abs(lowerShooterSpeed));
     }

    // public void inputRunShooter(){
    //     upperShooterMotor.set(Math.abs(inputUpperShooterSpeed.getDouble(0)));
    //     lowerShooterMotor.set(Math.abs(inputLowerShooterSpeed.getDouble(0)));
    // }
    


     public void runShooter(int RPM){

         if(RPM > Constants.Shooter.maxShooterRPM){
             RPM = Constants.Shooter.maxShooterRPM;
         }

         lowerPIDController.setReference(RPM, ControlType.kVelocity);
         upperPIDController.setReference(RPM, ControlType.kVelocity);
     }

    //detects ifthe shooter is at a certain range of RPM
    public boolean isShooterAtRPM(int RPM){

        if((getUpperShooterRPM() > (RPM - 30) && (getUpperShooterRPM() < (RPM + 30)))
        && (getLowerShooterRPM() > (RPM - 30) && (getLowerShooterRPM() < (RPM + 30)))){

            return true;
        }

        else{
            return false;
        }
    }

    public void stopShooter(){
        
        upperShooterMotor.stopMotor();
        lowerShooterMotor.stopMotor();
    }

    public double getUpperShooterValues(){
        return upperShooterMotor.get();
    }

    public double getUpperShooterRPM(){
        return upperShooterMotor.getEncoder().getVelocity();
    }

    public double getLowerShooterValues(){
        return lowerShooterMotor.get();
    }

    public double getLowerShooterRPM(){
        return lowerShooterMotor.getEncoder().getVelocity();
    }

    public double getInputShooterSpeed(){
        return inputShooterSpeed.getDouble(0);
    }

    @Override
    public void periodic(){

        Shuffleboard.update();
    


        // Shuffleboard.getTab("Shooter").add("upper shooter RPM", getUpperShooterRPM());
        // Shuffleboard.getTab("Shooter").add("lower shooter RPM", getLowerShooterRPM());
        // SmartDashboard.putBoolean("Note sensor value", getSensorValue());
        // SmartDashboard.putNumber("upper shooter value",getUpperShooterRPM());
        // SmartDashboard.putNumber("lower shooter value",getLowerShooterRPM());
    }


}