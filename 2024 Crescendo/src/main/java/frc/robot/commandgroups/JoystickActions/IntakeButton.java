package frc.robot.commandgroups.JoystickActions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commandgroups.IntakeWithArmCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeButton extends SequentialCommandGroup{

    public IntakeButton(ShooterSubsystem shooterSubsystem, ArmSubsystem armSubsystem){

        addCommands(
            new InstantCommand(() -> armSubsystem.extendArm()),
            new WaitCommand(1),
            new IntakeWithArmCommandGroup(shooterSubsystem)
        );
    }
    

}
