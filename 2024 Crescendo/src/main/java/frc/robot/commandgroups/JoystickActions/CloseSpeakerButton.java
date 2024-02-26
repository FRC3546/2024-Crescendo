package frc.robot.commandgroups.JoystickActions;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commandgroups.IntakeWithArmCommandGroup;
import frc.robot.commands.Arm.PIDRotateArmCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class CloseSpeakerButton extends SequentialCommandGroup{
    
    public CloseSpeakerButton(ArmSubsystem armSubsystem){

        addCommands(
            new InstantCommand(() -> armSubsystem.extendArm()),
            new PIDRotateArmCommand(() -> Constants.Arm.speakerArmAngle)
        );
    }
}
