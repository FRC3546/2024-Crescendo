package frc.robot.Auto;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.Shooter;
import frc.robot.commandgroups.IntakeWithArmCommandGroup;
import frc.robot.commands.Arm.PIDRotateArmCommand;
import frc.robot.commands.Arm.RotateArmCommand;
import frc.robot.commands.Intake.ReverseIntakeCommand;
import frc.robot.commands.Intake.SensorIntakeCommand;
import frc.robot.commands.Intake.SensorReverseIntakeCommand;
import frc.robot.commands.Intake.TimedIntakeCommand;
import frc.robot.commands.Shooter.PIDShooterCommand;
import frc.robot.commands.Shooter.RunShooterCommand;
import frc.robot.commands.Shooter.TimedRunShooterCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.Intake.SensorIntakeCommand;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoLeave extends SequentialCommandGroup{

    Optional<Alliance> ally = DriverStation.getAlliance();
    int redSide = -1;
    double vx = 1.5;
    double vy = 1.5;

    public AutoLeave(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem){

        if(ally.isPresent()){
            if (ally.get() == Alliance.Red) {
                vy = vy * redSide;                
            }
            if (ally.get() == Alliance.Blue) {

            }
        }
        
        addCommands(

        // get wheel in position
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new InstantCommand(() -> swerveSubsystem.lock()),
                    new WaitCommand(0.5),
                    new TimedDrive(swerveSubsystem, 1, 0, () -> 0, 2)),
                
                new RunShooterCommand(shooterSubsystem, () -> 0, () -> 0))
                        
        );
    }
    

}
