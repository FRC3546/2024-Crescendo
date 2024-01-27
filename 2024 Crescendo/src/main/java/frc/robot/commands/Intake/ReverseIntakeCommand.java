package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class ReverseIntakeCommand extends Command{

    IntakeShooterSubsystem intakeSubsystem;
    
    public ReverseIntakeCommand(IntakeShooterSubsystem intakeSubsystem){
        
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);

    }

    @Override
    public void initialize(){
        intakeSubsystem.reverseIntake();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

