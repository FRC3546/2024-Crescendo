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
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.Intake.SensorIntakeCommand;

public class TwoNoteAuto extends SequentialCommandGroup{


    public TwoNoteAuto(SwerveSubsystem swerveSubsystem, IntakeShooterSubsystem intakeShooterSubsystem){

        
        addCommands(

        // get wheel in position
            new InstantCommand(() -> swerveSubsystem.lock()),

            new ParallelRaceGroup(
                
                //
                new PIDRotateArmCommand(() -> Constants.Arm.testArmAngle),
                new ParallelDeadlineGroup(
                    new TimedRunShooterCommand(intakeShooterSubsystem, () -> 0.6, () -> 0.6, 3),
                    new SequentialCommandGroup(
                        new WaitCommand(0.7), 
                        new TimedIntakeCommand(intakeShooterSubsystem, 1, 1.5))
            )),

            
            new ParallelDeadlineGroup(
                new TimedDrive(swerveSubsystem, 1, 0, 0, 2.5),
                new IntakeWithArmCommandGroup()),
            
            new ParallelDeadlineGroup(
                new TimedDrive(swerveSubsystem, 1, 0, 0, 2.5),
                new PIDRotateArmCommand(() -> Constants.Arm.testArmAngle)
            ),

            new ParallelDeadlineGroup(
                    new TimedRunShooterCommand(intakeShooterSubsystem, () -> 0.6, () -> 0.6, 3),
                    new SequentialCommandGroup(
                        new WaitCommand(0.7), 
                        new TimedIntakeCommand(intakeShooterSubsystem, 1, 1.5)))
                        
        );
    }
    

}
