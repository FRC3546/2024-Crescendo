package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.ReverseIntakeCommand;
import frc.robot.commands.Intake.SensorIntakeCommand;
import frc.robot.commands.Intake.SensorReverseIntakeCommand;
import frc.robot.commands.Intake.TimedIntakeCommand;
import frc.robot.commands.Shooter.PIDShooterCommand;
import frc.robot.commands.Shooter.TimedRunShooterCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.Intake.SensorIntakeCommand;

public class AmpScoreCommandGroup extends SequentialCommandGroup{

    public AmpScoreCommandGroup(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem){

        addCommands(
            new ParallelDeadlineGroup(
                new TimedRunShooterCommand(shooterSubsystem, () -> 0.3, () -> 0.3, 2),
                new TimedIntakeCommand(intakeSubsystem, 1, 2) )
        );
    }
    

}
