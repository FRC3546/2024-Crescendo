package frc.robot.commandgroups.JoystickActions;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;
import frc.robot.commandgroups.IntakeWithArmCommandGroup;
import frc.robot.commands.Arm.PIDRotateArmCommand;
import frc.robot.commands.Shooter.RunShooterCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FarSpeakerButton extends SequentialCommandGroup{
    
    public FarSpeakerButton(ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, ClimbSubsystem climbSubsystem){

        addCommands(
            new InstantCommand(() -> armSubsystem.extendArm()),
            new ParallelDeadlineGroup(
                new PIDRotateArmCommand(() -> Constants.Arm.stageShotArmAngle),
                new RunShooterCommand(shooterSubsystem, climbSubsystem, () -> 0.75, () -> 0.75))
        );
    }
}
