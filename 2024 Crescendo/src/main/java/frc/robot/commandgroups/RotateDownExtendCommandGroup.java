// package frc.robot.commandgroups;

//  import edu.wpi.first.wpilibj2.command.PIDCommand;
//  import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//  import edu.wpi.first.wpilibj2.command.WaitCommand;
//  import frc.robot.Robot;
//  import frc.robot.RobotContainer;
//  import frc.robot.commands.Intake.ReverseIntakeCommand;
//  import frc.robot.commands.Intake.SensorIntakeCommand;
//  import frc.robot.commands.Intake.SensorReverseIntakeCommand;
//  import frc.robot.commands.Shooter.PIDShooterCommand;
//  import frc.robot.commands.Intake.SensorIntakeCommand;

//  public class RotateDownExtendCommandGroup extends SequentialCommandGroup{

//      public RotateDownExtendCommandGroup(){

//          addCommands(
//              new RotateCommand(RobotContainer.intakeShooterSubsystem, 0.6),
//              new WaitCommand(1),
//              new SensorReverseIntakeCommand(RobotContainer.intakeShooterSubsystem)
//          );
//      }
    

//  }