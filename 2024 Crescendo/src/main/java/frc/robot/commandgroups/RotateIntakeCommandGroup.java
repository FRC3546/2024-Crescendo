package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.PIDRotateArmCommand;
import frc.robot.subsystems.LedSubsystem;


public class RotateIntakeCommandGroup extends ParallelDeadlineGroup {

    public RotateIntakeCommandGroup(LedSubsystem ledSubsystem) {

        super(new PIDRotateArmCommand(() -> Constants.Arm.intakeArmAngle),
                new IntakeNoteCommandGroup(ledSubsystem));
    }
}
