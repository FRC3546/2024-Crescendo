package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.PIDRotateArmCommand;
import frc.robot.commands.Shooter.PIDShooterCommand;

public class RotateAmpCommandGroup extends ParallelDeadlineGroup {

    public RotateAmpCommandGroup() {

        super(new PIDRotateArmCommand(() -> Constants.Arm.ampArmAngle),
        new PIDShooterCommand(RobotContainer.shooterSubsystem, Constants.Shooter.ampRPM));

    }
}
