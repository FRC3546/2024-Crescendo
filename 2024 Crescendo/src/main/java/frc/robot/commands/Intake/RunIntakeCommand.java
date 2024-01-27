package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class RunIntakeCommand extends Command{

    IntakeShooterSubsystem intakeSubsystem;
    double speed;
    
    public RunIntakeCommand(IntakeShooterSubsystem intakeSubsystem, double speed){
        
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
        intakeSubsystem.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

