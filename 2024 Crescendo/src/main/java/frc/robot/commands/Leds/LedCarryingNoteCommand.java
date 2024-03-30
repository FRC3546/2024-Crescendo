package frc.robot.commands.Leds;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LedCarryingNoteCommand extends Command {

  private final LedSubsystem ledSubsystem;
  public IntakeSubsystem intakeSubsystem;

  public LedCarryingNoteCommand(LedSubsystem ledSubsystem, IntakeSubsystem intakeSubsystem) {

    this.intakeSubsystem = intakeSubsystem;
    this.ledSubsystem = ledSubsystem;


    addRequirements(ledSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (intakeSubsystem.getSecondSensorValue()) {
      ledSubsystem.green();
    } else {
      ledSubsystem.red();
    }
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