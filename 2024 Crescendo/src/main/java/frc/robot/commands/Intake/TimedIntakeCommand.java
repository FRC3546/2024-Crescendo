package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class TimedIntakeCommand extends Command{

    Timer timer = new Timer();
    IntakeShooterSubsystem intakeSubsystem;
    double speed;
    double time;
    
    public TimedIntakeCommand(IntakeShooterSubsystem intakeSubsystem, double speed, double time){
        
        this.speed = speed;
        this.time = time;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);

    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        intakeSubsystem.intake(speed);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake();
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return (timer.get() >= time);  
    }  
}

