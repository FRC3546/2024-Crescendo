package frc.robot.commandgroups.JoystickActions;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commandgroups.IntakeWithArmCommandGroup;
import frc.robot.commands.Arm.PIDRotateArmCommand;
import frc.robot.commands.Shooter.RunShooterCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpButton extends SequentialCommandGroup{

    public AmpButton(ShooterSubsystem shooterSubsystem, ArmSubsystem armSubsystem){

        addCommands(

            new InstantCommand(() -> armSubsystem.extendArm()),
            
            new ParallelDeadlineGroup(
                new PIDRotateArmCommand(() -> Constants.Arm.ampArmAngle),
                new RunShooterCommand(shooterSubsystem, () -> 0.15, () -> 0.15))
        );
    }
    

}
