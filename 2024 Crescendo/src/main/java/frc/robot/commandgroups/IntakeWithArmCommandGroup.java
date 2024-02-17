package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.PIDRotateArmCommand;

import frc.robot.commands.Intake.SensorIntakeCommand;


public class IntakeWithArmCommandGroup extends ParallelDeadlineGroup {

    public IntakeWithArmCommandGroup() {

        super(new SensorIntakeCommand(RobotContainer.intakeShooterSubsystem, 0.6),
                new PIDRotateArmCommand(Constants.Arm.intakeArmAngle));

    }
}
