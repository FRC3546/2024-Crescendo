package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

    private CANSparkMax leftClimbMotor;
    private CANSparkMax rightClimbMotor;
    private DoubleSolenoid climberPiston;


    public ClimbSubsystem() {

        climberPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);

        leftClimbMotor = new CANSparkMax(40, MotorType.kBrushless);
        rightClimbMotor = new CANSparkMax(41, MotorType.kBrushless);

        leftClimbMotor.follow(rightClimbMotor, true);
        rightClimbMotor.setInverted(true);

        leftClimbMotor.setSmartCurrentLimit(40);
        rightClimbMotor.setSmartCurrentLimit(40);

        leftClimbMotor.burnFlash();
        rightClimbMotor.burnFlash();

    }

    public void rotateClimbers(double speed) {

        rightClimbMotor.set(speed);
    }

    public void stopClimber(){

        rightClimbMotor.stopMotor();
    }

    public void extendClimberPiston(){

        climberPiston.set(Value.kForward);
    }

    public void retractClimberPiston(){

        climberPiston.set(Value.kReverse);
    }

    public void toggleClimberPiston(){

        climberPiston.toggle();
    }

    public double getRightEncoder(){
        return rightClimbMotor.getEncoder().getPosition();
    }

    public double getLeftEncoder(){
        return leftClimbMotor.getEncoder().getPosition();
    }
}