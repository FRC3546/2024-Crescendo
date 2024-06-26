package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.ReverseIntakeCommand;
import frc.robot.commands.Intake.SensorIntakeCommand;
import frc.robot.commands.Intake.SensorReverseIntakeCommand;
import frc.robot.commands.Leds.LedCarryingNoteCommand;
import frc.robot.commands.Shooter.PIDShooterCommand;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.commands.Intake.SensorIntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeNoteCommandGroup extends SequentialCommandGroup{

    public IntakeNoteCommandGroup(LedSubsystem ledSubsystem, IntakeSubsystem intakeSubsystem){

        addCommands(
            new SensorIntakeCommand(RobotContainer.intakeSubsystem, 1, 0.47),
            
            // new WaitCommand(0.1),
            new SensorReverseIntakeCommand(RobotContainer.intakeSubsystem)
            // new LedCarryingNoteCommand(ledSubsystem, intakeSubsystem));
        );
    }
    

}

