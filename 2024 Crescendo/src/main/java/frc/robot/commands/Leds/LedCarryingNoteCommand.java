package frc.robot.commands.Leds;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.LedSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LedCarryingNoteCommand extends Command{

    private final LedSubsystem ledSubsystem;

    public LedCarryingNoteCommand(LedSubsystem ledSubsystem){
       
        this.ledSubsystem = ledSubsystem;

        addRequirements(ledSubsystem);

    }

     // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    ledSubsystem.orange();
    new WaitCommand(0.5);
    ledSubsystem.off();
    new WaitCommand(0.5);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}