package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.SensorIntakeCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Shooter.PIDShooterCommand;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class AmpScoreCommandGroup extends ParallelCommandGroup{
    
    public SpeakerScoreCommandGroup(int RPM){

        addCommands(
            
        );
    }
    

}