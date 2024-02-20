package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.PIDRotateArmCommand;


public class RotateIntakeCommandGroup extends ParallelDeadlineGroup {

    public RotateIntakeCommandGroup() {

        super(new PIDRotateArmCommand(Constants.Arm.intakeArmAngle),
                new IntakeNoteCommandGroup());
    }
}
