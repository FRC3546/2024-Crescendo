package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeShooterSubsystem extends SubsystemBase{

    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    private GenericEntry inputUpperShooterSpeed = tab.add("upper shooter speed", 0).getEntry();
    private GenericEntry inputLowerShooterSpeed = tab.add("lower shooter speed", 0).getEntry();

    private VictorSP intakeMotor;
    private CANSparkMax upperShooterMotor;
    private CANSparkMax lowerShooterMotor;

    private double upperShooterSpeed;
    private double lowerShooterSpeed;


    public IntakeShooterSubsystem(){

        intakeMotor = new VictorSP(0);
        lowerShooterMotor = new CANSparkMax (31, MotorType.kBrushless);
        upperShooterMotor = new CANSparkMax (30, MotorType.kBrushless);
        
        intakeMotor.setInverted(true);

    }

    public void intake(double speed){
        
        intakeMotor.set(speed);

    }

    public void reverseIntake(){
        
        intakeMotor.set(-1);

    }

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
        lowerShooterMotor.set(Math.abs(inputUpperShooterSpeed.getDouble(0)));
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

    public double getLowerShooterRPM(){
        return lowerShooterMotor.getEncoder().getVelocity();
    }

    public double getLowerShooterValues(){
        return lowerShooterMotor.get();
    }
}