package frc.robot.commandgroups;

 import edu.wpi.first.wpilibj2.command.PIDCommand;
 import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
 import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
 import frc.robot.RobotContainer;
import frc.robot.Constants.Arm;
import frc.robot.commands.Arm.PIDRotateArmCommand;
import frc.robot.Constants.OperatorConstants;
 import frc.robot.commands.Intake.SensorIntakeCommand;
 import frc.robot.commands.Intake.SensorReverseIntakeCommand;
 import frc.robot.commands.Shooter.PIDShooterCommand;
 import frc.robot.subsystems.IntakeShooterSubsystem;


public class IntakeWithArmCommandGroup extends SequentialCommandGroup {
    
    public IntakeWithArmCommandGroup(){

        addCommands(
            new PIDRotateArmCommand(Constants.Arm.intakeArmAngle),
            new SensorIntakeCommand(RobotContainer.intakeShooterSubsystem, 0.6)
        );
            
        
    }
}
