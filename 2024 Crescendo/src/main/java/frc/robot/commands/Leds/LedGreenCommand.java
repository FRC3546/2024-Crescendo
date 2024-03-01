package frc.robot.commands.Leds;
import frc.robot.subsystems.LedSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LedGreenCommand extends Command{

    private final LedSubsystem ledSubsystem;

    public LedGreenCommand(LedSubsystem ledSubsystem){
       
        this.ledSubsystem = ledSubsystem;

        addRequirements(ledSubsystem);

    }

     // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledSubsystem.green();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
