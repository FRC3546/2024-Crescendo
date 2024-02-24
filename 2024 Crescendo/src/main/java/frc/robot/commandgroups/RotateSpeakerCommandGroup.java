package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.PIDRotateArmCommand;
import frc.robot.commands.Shooter.PIDShooterCommand;

public class RotateSpeakerCommandGroup extends ParallelDeadlineGroup {

    public RotateSpeakerCommandGroup() {

        super(new PIDRotateArmCommand(() -> 
        Constants.Arm.speakerArmAngle),
                new PIDShooterCommand(RobotContainer.shooterSubsystem, Constants.Shooter.speakerRPM));

    }
}
