package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.RunIntakeCommand;
import frc.robot.commands.Shooter.PIDShooterCommand;

public class FireNoteCommandGroup extends SequentialCommandGroup{

    public FireNoteCommandGroup(int RPM){

        addCommands(
            new PIDShooterCommand(RobotContainer.intakeShooterSubsystem, RPM),
            
            new RunIntakeCommand(RobotContainer.intakeShooterSubsystem, 1)
        );
    }
    

}
