package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Shooter.PIDShooterCommand;
import frc.robot.commands.Intake.IntakeCommand;

public class SpeakerScoreCommandGroup extends SequentialCommandGroup{

    public SpeakerScoreCommandGroup(int RPM){

        addCommands(
            new PIDShooterCommand(RobotContainer.intakeShooterSubsystem, RPM),
            
            new IntakeCommand(RobotContainer.intakeShooterSubsystem, 1)
        );
    }
    

}
