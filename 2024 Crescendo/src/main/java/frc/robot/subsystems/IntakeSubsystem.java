package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase{

    private VictorSP intakeMotor;
    private CANSparkMax upperShooterMotor;
    private CANSparkMax lowerShooterMotor;

}

private IntakeSubsystem(){

    intakeMotor = new VictorSP(0);
    upperShooterMotor = new CANSparkMax (31, MotorType.kBrushless);
    lowerShooterMotor = new CANSparkMax (30, MotorType.kBrushless);

}

public void intake(){
    
    intakeMotor.set(1);

}

public void reverseIntake(){
    
    intakeMotor.set(-1);

}

public void stopIntake(){

    intakeMotor.set(0);

}

public void runShooter(){
    
    upperShooterMotor.set(1);
    lowerShooterMotor.set(1);

}

public void stopShooter(){
    
    upperShooterMotor.set(0);
    lowerShooterMotor.set(0);

}
}