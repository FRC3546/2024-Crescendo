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
import frc.robot.commands.Intake.ReverseIntakeCommand;
import frc.robot.commands.Intake.SensorIntakeCommand;
import frc.robot.commands.Intake.SensorReverseIntakeCommand;
import frc.robot.commands.Intake.TimedIntakeCommand;
import frc.robot.commands.Limelight.AmpLineupCommand;
import frc.robot.commands.PhotonVision.TranslateToNoteCommand;
import frc.robot.commands.Shooter.PIDShooterCommand;
import frc.robot.commands.Shooter.RunShooterCommand;
import frc.robot.commands.Shooter.TimedRunShooterCommand;
import frc.robot.commands.Swerve.RotateToAngle;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.Intake.SensorIntakeCommand;


import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ThreeNoteCenter extends SequentialCommandGroup{

    private int blueMultiplier;

    public ThreeNoteCenter(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, ArmSubsystem armSubsystem, ClimbSubsystem climbSubsystem, LedSubsystem ledSubsystem, PhotonVisionSubsystem photonVisionSubsystem, LimelightSubsystem limelightSubsystem, boolean isRed){

        blueMultiplier = isRed ? 1 : -1;
        // shooterSubsystem.setDefaultCommand(new RunShooterCommand(shooterSubsystem, climbSubsystem, () -> 0.6,  () -> 0.6));
        addCommands(

        // get wheel in position
            

            new InstantCommand(() -> swerveSubsystem.lock()),


            new TimedDrive(swerveSubsystem, 0, -0.5, 0, 1),

            new RotateToAngle(swerveSubsystem, () -> 0).withTimeout(1),

            new ParallelDeadlineGroup(
                new TimedDriveGyro(swerveSubsystem, -1, 0, () -> 0, 0.6),
                new PIDRotateArmCommand(() -> Constants.Arm.speakerArmAngle)
            ),

            new ParallelRaceGroup(
    
                new PIDRotateArmCommand(() -> Constants.Arm.speakerArmAngle),
                new ParallelDeadlineGroup(
                    new TimedRunShooterCommand(shooterSubsystem, () -> 0.65, () -> 0.65, 1.5),
                    new SequentialCommandGroup(
                        new WaitCommand(0.75), 
                        new TimedIntakeCommand(intakeSubsystem, 1, 0.75))
            )),

            new ParallelDeadlineGroup(
                new TimedDriveGyro(swerveSubsystem, 1.5, 0, () -> 0, 1.2),
                new IntakeButton(shooterSubsystem, armSubsystem, intakeSubsystem, ledSubsystem, climbSubsystem, () -> false)                       
            ),

            new ParallelDeadlineGroup(
                new TimedDriveGyro(swerveSubsystem, -1.6, 0, () -> 0, 1.5),
                new PIDRotateArmCommand(() -> Constants.Arm.speakerArmAngle),
                new TimedRunShooterCommand(shooterSubsystem, () -> 0.65, () -> 0.65, 1.5)
            ),
            new ParallelRaceGroup(
    
                new ParallelDeadlineGroup(
                    new TimedRunShooterCommand(shooterSubsystem, () -> 0.65, () -> 0.65, 0.5),
                    new TimedIntakeCommand(intakeSubsystem, 1, 0.75))
            ),

            new TimedDrive(swerveSubsystem, 0, 2 * blueMultiplier, 0, 1),
            new TranslateToNoteCommand(swerveSubsystem, photonVisionSubsystem, 12).withTimeout(1.5),

            new ParallelDeadlineGroup(
                new TimedDriveGyro(swerveSubsystem, 0.75, 0, () -> 0, 1.75),
                new IntakeButton(shooterSubsystem, armSubsystem, intakeSubsystem, ledSubsystem, climbSubsystem, null)
            ),

            new TimedDriveGyro(swerveSubsystem, -2.5, 0, () -> 0, 0.25),

            new RotateToAngle(swerveSubsystem, () -> (40 * blueMultiplier)).withTimeout(1),

            new ParallelDeadlineGroup(
                // new AmpLineupCommand(limelightSubsystem, swerveSubsystem, ledSubsystem, -3.9, 16, () -> 0).withTimeout(1),
                new TimedDrive(swerveSubsystem, -2, 0, 0, 1),
                new PIDRotateArmCommand(() -> Constants.Arm.speakerArmAngle),
                new TimedRunShooterCommand(shooterSubsystem, () -> 0.65, () -> 0.65, 1)

            ),

            new ParallelDeadlineGroup(
                new TimedIntakeCommand(intakeSubsystem, 1, 0.5),
                new TimedRunShooterCommand(shooterSubsystem, () -> 0.65, () -> 0.65, 0.5)
            )
            // new ParallelRaceGroup(
    
            //     new PIDRotateArmCommand(() -> Constants.Arm.speakerArmAngle),
            //     new ParallelDeadlineGroup(
            //         new WaitCommand(1.5)
            //         // new TimedRunShooterCommand(shooterSubsystem, () -> 0.6, () -> 0.6, 1.5),
            //         // new SequentialCommandGroup(
            //         //     new WaitCommand(0.75), 
            //         //     new TimedIntakeCommand(intakeSubsystem, 1, 0.75))
            // )),

            // new RunShooterCommand(shooterSubsystem, climbSubsystem, () -> 0, () -> 0)

        );
    }
    

}
