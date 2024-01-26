package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class ReverseIntakeCommand extends Command{

    IntakeShooterSubsystem reverseIntakeSubsystem;
    
    public ReverseIntakeCommand(IntakeShooterSubsystem shooterSubsystem){
        
        this.reverseIntakeSubsystem = reverseIntakeSubsystem;
        addRequirements(reverseIntakeSubsystem);

    }

    @Override
    public void initialize(){
        reverseIntakeSubsystem.reverseIntake();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        reverseIntakeSubsystem.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

