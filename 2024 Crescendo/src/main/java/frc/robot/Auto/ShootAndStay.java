package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
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
import frc.robot.commands.Swerve.RotateToAngle;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.Intake.SensorIntakeCommand;


import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ShootAndStay extends SequentialCommandGroup{

    public ShootAndStay(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, ArmSubsystem armSubsystem, ClimbSubsystem climbSubsystem){

        addCommands(

        // get wheel in position
            new InstantCommand(() -> swerveSubsystem.lock()),

            new ParallelDeadlineGroup(
                new WaitCommand(2.5),
                new RunShooterCommand(shooterSubsystem, climbSubsystem, () -> 0.6, () -> 0.6)
            ),

            new ParallelRaceGroup(
                
                //
                new PIDRotateArmCommand(() -> Constants.Arm.speakerArmAngle),
                new ParallelDeadlineGroup(
                    new TimedRunShooterCommand(shooterSubsystem, () -> 0.6, () -> 0.6, 3),
                    new SequentialCommandGroup(
                        new WaitCommand(1), 
                        new TimedIntakeCommand(intakeSubsystem, 1, 2))
            ))

        );
    }
    

}
