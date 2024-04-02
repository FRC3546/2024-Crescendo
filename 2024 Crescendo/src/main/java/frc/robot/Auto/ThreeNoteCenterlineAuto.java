package frc.robot.Auto;

import java.util.Optional;

import javax.naming.PartialResultException;

import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.commandgroups.IntakeWithArmCommandGroup;
import frc.robot.commandgroups.JoystickActions.IntakeButton;
import frc.robot.commands.Arm.PIDRotateArmCommand;
import frc.robot.commands.Arm.RotateArmCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Intake.ReverseIntakeCommand;
import frc.robot.commands.Intake.SensorIntakeCommand;
import frc.robot.commands.Intake.SensorReverseIntakeCommand;
import frc.robot.commands.Intake.TimedIntakeCommand;
import frc.robot.commands.Limelight.TargetOnTheMove;
import frc.robot.commands.PhotonVision.RotateToNoteCommand;
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

public class ThreeNoteCenterlineAuto extends SequentialCommandGroup{

    private int blueMuliplier;

    public ThreeNoteCenterlineAuto(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, LedSubsystem ledSubsystem, ArmSubsystem armSubsystem, ClimbSubsystem climbSubsystem, PhotonVisionSubsystem photonVisionSubsystem, LimelightSubsystem limelightSubsystem, boolean isRed){
        
        blueMuliplier = isRed ? 1 : -1;
        double timeToCenterline = 1.66;
        double firstLeg = 0.73;
        double secondLeg = 0.6075;
        double thirdLeg = 0.9225;
        // double firstLeg = 0.5;
        // double secondLeg = 0.6;
        // double thirdLeg = 0.7;
        double centerLineAngle = -1.5;

        double driveSpeed = 3.95;

        // double timeToFirstLeg = timeToCenterline * firstLeg;
        // double timeToSecondLeg = secondLeg * timeToCenterline - timeToFirstLeg;
        // double timeToThirdLeg

        addCommands(

        // get wheel in position
            
            
            // scoring
            new ParallelDeadlineGroup(
                new RotateToAngle(swerveSubsystem, () -> -46.32 * blueMuliplier).withTimeout(3.75),
                new PIDRotateArmCommand(() -> Constants.Arm.speakerArmAngle + 0.009721666),
                new ParallelDeadlineGroup(
                    new TimedRunShooterCommand(shooterSubsystem, () -> 0.6, () -> 0.6, 3.75),
                    new SequentialCommandGroup(
                        new WaitCommand(2.25), 
                        new TimedIntakeCommand(intakeSubsystem, 1, 1.5))
            )),

            // new ParallelDeadlineGroup(
            //     new TimedIntakeCommand(intakeSubsystem, 1, 1.5),
            //     new TimedRunShooterCommand(shooterSubsystem, () -> 0.6, () -> 0.6, 4)),

            // rotate to angle 0
            new ParallelDeadlineGroup(
                new RotateToAngle(swerveSubsystem, () -> 0 * blueMuliplier).withTimeout(1),
                new PIDRotateArmCommand(() -> Constants.Arm.intakeArmAngle)
            ),

            // // rotate to angle 0
            // new ParallelDeadlineGroup(
            //     new WaitCommand(2),
            //     new RotateToAngle(swerveSubsystem, () -> (0)),
            //     new PIDRotateArmCommand(() -> Constants.Arm.intakeArmAngle)
            //     ),
            
            // backup and pick up note
            new ParallelRaceGroup(
                new TimedDriveGyro(swerveSubsystem, 2, 0, () -> 0 * blueMuliplier, 1),
                new IntakeButton(shooterSubsystem, armSubsystem, intakeSubsystem, ledSubsystem, climbSubsystem, () -> false)
                ),

            new ParallelDeadlineGroup(new WaitCommand(0.4), new IntakeCommand(intakeSubsystem, 1)),

            new ParallelDeadlineGroup(
                new WaitCommand(0.5),
                new SensorReverseIntakeCommand(intakeSubsystem)),

            new ParallelDeadlineGroup(
                new RotateToAngle(swerveSubsystem, () -> -25 * blueMuliplier).withTimeout(3),
                //Stage shot angle plus offset to make note not miss high
                new PIDRotateArmCommand(() -> Constants.Arm.stageShotArmAngle - 0.0023),
                new ParallelDeadlineGroup(
                    new TimedRunShooterCommand(shooterSubsystem, () -> 0.75, () -> 0.75, 3),
                    new SequentialCommandGroup(
                        new WaitCommand(2.5), 
                        new TimedIntakeCommand(intakeSubsystem, 1, 2.5))
            )),

            // new TimedDrive(swerveSubsystem, 2.5, 0, 0, 0.5),

            new RotateToAngle(swerveSubsystem, () -> centerLineAngle).withTimeout(1),

            // 1st leg
            new ParallelRaceGroup(
                new TimedDriveGyro(swerveSubsystem, driveSpeed, 0, () -> centerLineAngle * blueMuliplier, firstLeg),
                new PIDRotateArmCommand(() -> Constants.Arm.speakerArmAngle)
            ),

            // 2nd leg
            new TimedDriveGyro(swerveSubsystem, driveSpeed/2, 0, () -> ((centerLineAngle * blueMuliplier) + photonVisionSubsystem.getX()), secondLeg),

            // 3rd leg
            new ParallelRaceGroup(
                new TimedDriveGyro(swerveSubsystem, driveSpeed, 0, () -> centerLineAngle * blueMuliplier, thirdLeg),
                new IntakeButton(shooterSubsystem, armSubsystem, intakeSubsystem, ledSubsystem, climbSubsystem, () -> false)
            ),

            new ParallelRaceGroup(
                new TimedDriveGyro(swerveSubsystem, -driveSpeed, 0, () -> centerLineAngle * blueMuliplier, 2.1),
                new RunShooterCommand(shooterSubsystem, climbSubsystem, () -> 0.75, () -> 0.75)
            ),

            new TargetOnTheMove(limelightSubsystem, swerveSubsystem, ledSubsystem,() -> 0, () -> 0, () -> -6.9).withTimeout(3),

            new ParallelDeadlineGroup(
                new RotateToAngle(swerveSubsystem, () -> -25 * blueMuliplier).withTimeout(3),
                //Stage shot angle plus offset to make note not miss high
                new PIDRotateArmCommand(() -> Constants.Arm.stageShotArmAngle - 0.0023),
                new ParallelDeadlineGroup(
                    new TimedRunShooterCommand(shooterSubsystem, () -> 0.75, () -> 0.75, 3),
                    new SequentialCommandGroup(
                        new WaitCommand(2.5), 
                        new TimedIntakeCommand(intakeSubsystem, 1, 2.5))
            ))





            
        

            

            // new ParallelRaceGroup(
            //     new TimedDriveGyro(swerveSubsystem, -2.75, 0, () -> 0 * blueMuliplier, 1.5),
            //     new IntakeButton(shooterSubsystem, armSubsystem, intakeSubsystem, ledSubsystem, () -> false)
            //     )

            

            

            // new ParallelDeadlineGroup(
            //     new TimedIntakeCommand(intakeSubsystem, 1, 1.5),
            //     new TimedRunShooterCommand(shooterSubsystem, () -> 0.75, () -> 0.75, 4))
            
            // aim at speaker
            // new ParallelDeadlineGroup(
            //     new WaitCommand(1.5),
            //     new RotateToAngle(swerveSubsystem, () -> (-18))),

            // new TimedDrive(swerveSubsystem, -1, 0, () -> -18, 1),

            //shoot
            // new ParallelRaceGroup(
                
            
            //     new PIDRotateArmCommand(() -> (Constants.Arm.stageShotArmAngle + 0.01)),
            //     new ParallelDeadlineGroup(
            //         new TimedRunShooterCommand(shooterSubsystem, () -> 0.75, () -> 0.75, 4),
            //         new SequentialCommandGroup(
            //             new WaitCommand(2.5), 
            //             new TimedIntakeCommand(intakeSubsystem, 1, 1.5))
            // ))
                        
        );
    }
    

}
