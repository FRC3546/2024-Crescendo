package frc.robot.commandgroups.JoystickActions;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commandgroups.IntakeWithArmCommandGroup;
import frc.robot.commands.Arm.PIDRotateArmCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StowedButton extends SequentialCommandGroup{

    public StowedButton(ShooterSubsystem shooterSubsystem, ArmSubsystem armSubsystem){

        addCommands(

            new ParallelDeadlineGroup(
                new PIDRotateArmCommand(() -> Constants.Arm.speakerArmAngle),
                new SequentialCommandGroup(
                    new WaitCommand(1),
                    new InstantCommand(() -> armSubsystem.retractArm())))
            
        );
    }
    

}
