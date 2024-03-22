package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.JoystickRotateArmCommand;
import frc.robot.commands.Arm.PIDManualArmCommand;


public class ManualArmControlCommandGroup extends ParallelCommandGroup {

    public ManualArmControlCommandGroup() {

        addCommands(

        new JoystickRotateArmCommand(() -> RobotContainer.shooterJoystick.getRawAxis(1), RobotContainer.ledSubsystem),
        new PIDManualArmCommand()
        
        );

    }

}