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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.Intake.SensorIntakeCommand;


import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class OneNoteLoadSideAuto extends SequentialCommandGroup{

    Optional<Alliance> ally = DriverStation.getAlliance();
    int redSide = -1;
    double vx = 1.5;
    double vy = 1.5;


    public OneNoteLoadSideAuto(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, ArmSubsystem armSubsystem){

        if(ally.isPresent()){
            if (ally.get() == Alliance.Red) {
                vy = vy * redSide;                
            }
            if (ally.get() == Alliance.Blue) {

            }
        }
        
        addCommands(

        // get wheel in position
            new InstantCommand(() -> armSubsystem.extendArm()),
            new InstantCommand(() -> swerveSubsystem.lock()),

            new ParallelDeadlineGroup(
                new WaitCommand(2.5),
                new RunShooterCommand(shooterSubsystem, () -> 0.6, () -> 0.6)
            ),

            new ParallelRaceGroup(
                
                //
                new PIDRotateArmCommand(() -> Constants.Arm.speakerArmAngle),
                new ParallelDeadlineGroup(
                    new TimedRunShooterCommand(shooterSubsystem, () -> 0.6, () -> 0.6, 3),
                    new SequentialCommandGroup(
                        new WaitCommand(1), 
                        new TimedIntakeCommand(intakeSubsystem, 1, 2))
            )),

            // new TimedDrive(swerveSubsystem, 0, 0, 1, 1.5),

            new TimedDrive(swerveSubsystem, vx, 0, 0, 2),

            // new InstantCommand(() -> swerveSubsystem.lock()),
            // new WaitCommand(0.5),
            new ParallelDeadlineGroup(
                new WaitCommand(4),
                new RotateToAngle(swerveSubsystem, () -> 0)),

            
            // new TimedDrive(swerveSubsystem, 0, 0, 1, 1.5),

            new TimedDrive(swerveSubsystem, vx, 0, 0, 3)


            // new TimedDrive(swerveSubsystem, 0, vy, 0, 2)
            
            // new ParallelDeadlineGroup(
            //     new TimedDrive(swerveSubsystem, 1, 0, 0, 2),
            //     new IntakeWithArmCommandGroup(shooterSubsystem)),
            
            // new ParallelDeadlineGroup(
            //     new TimedDrive(swerveSubsystem, 1, 0, 0, 2.5),
            //     new PIDRotateArmCommand(() -> Constants.Arm.testArmAngle)
            // ),

            // new ParallelDeadlineGroup(
            //         new TimedRunShooterCommand(shooterSubsystem, () -> 0.6, () -> 0.6, 3),
            //         new SequentialCommandGroup(
            //             new WaitCommand(0.7), 
            //             new TimedIntakeCommand(intakeSubsystem, 1, 1.5)))
                        
        );
    }
    

}
