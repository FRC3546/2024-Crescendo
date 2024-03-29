package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

public class ClimbSubsystem extends SubsystemBase {

    private DigitalInput climberLimitSwitch;
    private CANSparkMax leftClimbMotor;
    private CANSparkMax rightClimbMotor;
    private DoubleSolenoid climberPiston;
    private boolean climberReleased = false;


    public ClimbSubsystem() {


        climberLimitSwitch = new DigitalInput(2);

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
        climberReleased = true;
        climberPiston.set(Value.kForward);
    }

    public void retractClimberPiston(){
        climberReleased = false;
        climberPiston.set(Value.kReverse);
    }

    public void toggleClimberPiston(){

        climberReleased = !climberReleased;
        climberPiston.toggle();
    }

    public double getRightEncoder(){
        return rightClimbMotor.getEncoder().getPosition();
    }

    public double getLeftEncoder(){
        return leftClimbMotor.getEncoder().getPosition();
    }

    public boolean getLimitSwitchValue(){
        return climberLimitSwitch.get();
    }

    public boolean isClimberReleased(){
        return climberReleased;
    }
}