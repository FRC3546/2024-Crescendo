package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Shooter;
import frc.robot.commands.Arm.PIDRotateArmCommand;

import frc.robot.commands.Intake.SensorIntakeCommand;
import frc.robot.commands.Shooter.RunShooterCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeWithArmCommandGroup extends ParallelDeadlineGroup {
    
    

    public IntakeWithArmCommandGroup(ShooterSubsystem shooterSubsystem) {

        super(
            new PIDRotateArmCommand(() -> Constants.Arm.intakeArmAngle),
            new RunShooterCommand(shooterSubsystem, () -> 0, () -> 0),
            new IntakeNoteCommandGroup());
    }
}
