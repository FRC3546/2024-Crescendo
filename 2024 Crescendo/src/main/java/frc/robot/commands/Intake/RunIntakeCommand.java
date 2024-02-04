package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

        // intakeSubsystem.intake(-0.3);
        // new WaitCommand(0.5);
        intakeSubsystem.stopIntake();
    }

    @Override
    public boolean isFinished() {
        // return false;
        return intakeSubsystem.getSensorValue();
    }
}

