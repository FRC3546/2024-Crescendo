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
import frc.robot.commands.Limelight.AmpLineupCommand;
import frc.robot.commands.Shooter.PIDShooterCommand;
import frc.robot.commands.Swerve.DrivetoUltrasonicRangeCommand;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;


public class LineupAmpCommandGroup extends SequentialCommandGroup{

    public LineupAmpCommandGroup(SwerveSubsystem swerveSubsystem, LimelightSubsystem limelightSubsystem, LedSubsystem ledSubsystem){

        addCommands(
            new AmpLineupCommand(limelightSubsystem, swerveSubsystem, ledSubsystem, -7.5, 16, () -> 0),
            new DrivetoUltrasonicRangeCommand(swerveSubsystem, ledSubsystem, 12.5, () -> 0, () -> 0)
            );
    }
}