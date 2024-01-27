package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeShooterSubsystem extends SubsystemBase{

    private VictorSP intakeMotor;
    private CANSparkMax upperShooterMotor;
    private CANSparkMax lowerShooterMotor;

    private double upperShooterSpeed;
    private double lowerShooterSpeed;


    public IntakeShooterSubsystem(){

        intakeMotor = new VictorSP(0);
        lowerShooterMotor = new CANSparkMax (30, MotorType.kBrushless);
        upperShooterMotor = new CANSparkMax (31, MotorType.kBrushless);
        

    }

    public void intake(){
        
        intakeMotor.set(1);

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

        upperShooterMotor.set(-(Math.abs(upperShooterSpeed)));
        lowerShooterMotor.set(Math.abs(lowerShooterSpeed));

    }

    public void stopShooter(){
        
        upperShooterMotor.stopMotor();
        lowerShooterMotor.stopMotor();;

    }

    public double getUpperShooterValues(){
        return upperShooterMotor.get();
    }

    public double getLowerShooterValues(){
        return lowerShooterMotor.get();
    }
}