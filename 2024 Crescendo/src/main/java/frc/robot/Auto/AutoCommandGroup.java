package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.ReverseIntakeCommand;
import frc.robot.commands.Intake.SensorIntakeCommand;
import frc.robot.commands.Intake.SensorReverseIntakeCommand;
import frc.robot.commands.Shooter.PIDShooterCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.Intake.SensorIntakeCommand;

public class AutoCommandGroup extends SequentialCommandGroup{


    public AutoCommandGroup(SwerveSubsystem swerveSubsystem){

        
        addCommands(
            new InstantCommand(() -> swerveSubsystem.lock()),
            new WaitCommand(0.25),
            new TimedDrive(swerveSubsystem, 1, 0, 0, 3)
        );
    }
    

}
