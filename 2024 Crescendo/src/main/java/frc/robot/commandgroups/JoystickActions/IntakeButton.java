package frc.robot.commandgroups.JoystickActions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commandgroups.IntakeWithArmCommandGroup;
import frc.robot.commands.Shooter.RunShooterCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.BooleanSupplier;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeButton extends SequentialCommandGroup {

    public IntakeButton(
            ShooterSubsystem shooterSubsystem,
            ArmSubsystem armSubsystem,
            IntakeSubsystem intakeSubsystem,
            LedSubsystem ledSubsystem,
            ClimbSubsystem climbSubsystem,
            BooleanSupplier armExtended) {

        if (armExtended.getAsBoolean()) {
            System.out.println(armSubsystem.getArmExtension());
            addCommands(
                    new IntakeWithArmCommandGroup(shooterSubsystem, ledSubsystem, intakeSubsystem, climbSubsystem));
        }

        else {
            System.out.println("no " + armSubsystem.getArmExtension());
            addCommands(

                    // new InstantCommand(() -> armSubsystem.extendArm()),
                    // new WaitCommand(0.3),
                    new IntakeWithArmCommandGroup(shooterSubsystem, ledSubsystem, intakeSubsystem, climbSubsystem)

            // new ParallelDeadlineGroup(
            // new StowedButton(shooterSubsystem, armSubsystem),
            // new RunShooterCommand(shooterSubsystem, () -> 0.6, () -> 0.6))
            );
        }
    }

}
