package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.PIDRotateArmCommand;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class RotateIntakeCommandGroup extends ParallelDeadlineGroup {

    public RotateIntakeCommandGroup(LedSubsystem ledSubsystem, IntakeSubsystem intakeSubsystem) {

        super(new PIDRotateArmCommand(() -> Constants.Arm.intakeArmAngle),
                new IntakeNoteCommandGroup(ledSubsystem, intakeSubsystem));
    }
}
