package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeShooterSubsystem extends SubsystemBase{

    //dashboard
    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    private GenericEntry inputUpperShooterSpeed = tab.add("upper shooter speed", 0).getEntry();
    private GenericEntry inputLowerShooterSpeed = tab.add("lower shooter speed", 0).getEntry();



    //motors
    private VictorSP intakeMotor;
    private CANSparkMax upperShooterMotor;
    private CANSparkMax lowerShooterMotor;

    private SparkPIDController lowerPIDController;
    private SparkPIDController upperPIDController;

    DigitalInput noteSensor;

    private double upperShooterSpeed;
    private double lowerShooterSpeed;
    


    public IntakeShooterSubsystem(){

        intakeMotor = new VictorSP(0);
        lowerShooterMotor = new CANSparkMax (31, MotorType.kBrushless);
        upperShooterMotor = new CANSparkMax (30, MotorType.kBrushless);
        
        noteSensor = new DigitalInput(0);
        
        
        intakeMotor.setInverted(true);

        upperShooterMotor.restoreFactoryDefaults();
        lowerShooterMotor.restoreFactoryDefaults();

        lowerPIDController = lowerShooterMotor.getPIDController();
        upperPIDController = upperShooterMotor.getPIDController();

        lowerPIDController.setP(0.001);
        lowerPIDController.setI(0);
        lowerPIDController.setD(0.0001);
        lowerPIDController.setIZone(0);
        lowerPIDController.setFF(0.0002);
        lowerPIDController.setOutputRange(0, 1);

        upperPIDController.setP(0.0008);
        upperPIDController.setI(0);
        upperPIDController.setD(0.000);
        upperPIDController.setIZone(0);
        upperPIDController.setFF(0.0002);
        upperPIDController.setOutputRange(0, 1);
        
        lowerShooterMotor.burnFlash();
        upperShooterMotor.burnFlash();

        // Shuffleboard.getTab("Shooter").add("upper shooter RPM", getUpperShooterRPM());
        // Shuffleboard.getTab("Shooter").add("lower shooter RPM", getLowerShooterRPM());
        // Shuffleboard.getTab("Shooter").add("sensor value", noteSensor);
        // Shuffleboard.getTab("Shooter").add("note sensor boolean", getSensorValue());

    }

    public void intake(double speed){
        
        intakeMotor.set(speed);

    }

    // public void reverseIntake(){
        
    //     intakeMotor.set(-1);

    // }

    public void stopIntake(){

        intakeMotor.stopMotor();
    }

    public void runShooter(double upperShooterSpeed, double lowerShooterSpeed){
        
        this.upperShooterSpeed = upperShooterSpeed;
        this.lowerShooterSpeed = lowerShooterSpeed;

        upperShooterMotor.set(Math.abs(upperShooterSpeed));
        lowerShooterMotor.set(Math.abs(lowerShooterSpeed));
    }

    public void inputRunShooter(){
        upperShooterMotor.set(Math.abs(inputUpperShooterSpeed.getDouble(0)));
        lowerShooterMotor.set(Math.abs(inputLowerShooterSpeed.getDouble(0)));
    }

    public boolean getSensorValue(){
        
        // the note sensor returns true by default, so the value needs to be 
        // flipped so it returns true when detecting a piece
        return !noteSensor.get();
    }

    public void runShooter(int RPM){

        if(RPM > Constants.Shooter.maxShooterRMP){
            RPM = Constants.Shooter.maxShooterRMP;
        }

        lowerPIDController.setReference(RPM, ControlType.kVelocity);
        upperPIDController.setReference(RPM, ControlType.kVelocity);
    }

    public boolean isShooterAtRPM(int RPM){

        

        if((getUpperShooterRPM() > (RPM - 30) && (getUpperShooterRPM() < (RPM + 60)))
        && (getLowerShooterRPM() > (RPM - 30) && (getLowerShooterRPM() < (RPM + 60)))){

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

    @Override
    public void periodic(){

        Shuffleboard.update();
    


        // Shuffleboard.getTab("Shooter").add("upper shooter RPM", getUpperShooterRPM());
        // Shuffleboard.getTab("Shooter").add("lower shooter RPM", getLowerShooterRPM());
        // SmartDashboard.putBoolean("Note sensor value", getSensorValue());
        // SmartDashboard.putNumber("upper shooter value",RobotContainer.intakeShooterSubsystem.getUpperShooterRPM());
        // SmartDashboard.putNumber("lower shooter value",RobotContainer.intakeShooterSubsystem.getLowerShooterRPM());
    }


}