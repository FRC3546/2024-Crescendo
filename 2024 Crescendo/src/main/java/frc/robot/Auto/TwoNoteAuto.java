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
import frc.robot.commandgroups.JoystickActions.IntakeButton;
import frc.robot.commands.Arm.PIDRotateArmCommand;
import frc.robot.commands.Arm.RotateArmCommand;
import frc.robot.commands.Intake.IntakeCommand;
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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.Intake.SensorIntakeCommand;

public class TwoNoteAuto extends SequentialCommandGroup{


    public TwoNoteAuto(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, LedSubsystem ledSubsystem, ArmSubsystem armSubsystem){

        
        addCommands(

        // get wheel in position
            new InstantCommand(() -> armSubsystem.extendArm()),
            new InstantCommand(() -> swerveSubsystem.lock()),

            new WaitCommand(1),
            // scoring
            new ParallelRaceGroup(
                new PIDRotateArmCommand(() -> Constants.Arm.speakerArmAngle),
                new ParallelDeadlineGroup(
                    new TimedRunShooterCommand(shooterSubsystem, () -> 0.6, () -> 0.6, 3),
                    new SequentialCommandGroup(
                        new WaitCommand(1.5), 
                        new TimedIntakeCommand(intakeSubsystem, 1, 1.5))
            )),

            // rotate to angle 0
            new ParallelDeadlineGroup(
                new WaitCommand(2),
                new RotateToAngle(swerveSubsystem, () -> -5),
                new PIDRotateArmCommand(() -> Constants.Arm.intakeArmAngle)
                ),
            
            // backup and pick up note
            new ParallelRaceGroup(
                new TimedDrive(swerveSubsystem, 2, 0, () -> 0, 1.25),
                new IntakeButton(shooterSubsystem, armSubsystem, ledSubsystem)
                ),

            new ParallelDeadlineGroup(new WaitCommand(0.4), new IntakeCommand(intakeSubsystem, 1)),

            new ParallelDeadlineGroup(
                new WaitCommand(0.5),
                new SensorReverseIntakeCommand(intakeSubsystem)),
            
            // aim at speaker
            new ParallelDeadlineGroup(
                new WaitCommand(1.5),
                new RotateToAngle(swerveSubsystem, () -> -18)),

            // new TimedDrive(swerveSubsystem, -1, 0, () -> -18, 1),

            //shoot
            new ParallelRaceGroup(
                
            
                new PIDRotateArmCommand(() -> (Constants.Arm.stageShotArmAngle)),
                new ParallelDeadlineGroup(
                    new TimedRunShooterCommand(shooterSubsystem, () -> 0.75, () -> 0.75, 4),
                    new SequentialCommandGroup(
                        new WaitCommand(2.5), 
                        new TimedIntakeCommand(intakeSubsystem, 1, 1.5))
            ))
                        
        );
    }
    

}
