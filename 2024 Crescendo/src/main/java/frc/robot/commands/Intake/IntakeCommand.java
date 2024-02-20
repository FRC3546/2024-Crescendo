package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class IntakeCommand extends Command{

    IntakeShooterSubsystem intakeSubsystem;
    double speed;
    
    public IntakeCommand(IntakeShooterSubsystem intakeSubsystem, double speed){
        
        this.speed = speed;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);

    }

    @Override
    public void initialize(){
        intakeSubsystem.intake(speed);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
      
    }

    @Override
    public boolean isFinished() {
     new WaitCommand(2);
        intakeSubsystem.stopIntake(); 
        return true;  
    }  
}

