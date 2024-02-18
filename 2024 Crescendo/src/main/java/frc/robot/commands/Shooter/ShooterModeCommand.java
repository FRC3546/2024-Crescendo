// package frc.robot.commands.Shooter;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.RobotContainer;
// import frc.robot.commands.Intake.SensorIntakeCommand;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.IntakeShooterSubsystem;

// public class ShooterModeCommand extends Command {

//     IntakeShooterSubsystem shooterSubsystem;

//     public ShooterModeCommand(IntakeShooterSubsystem shooterSubsystem){
//         this.shooterSubsystem = shooterSubsystem;
//         addRequirements(shooterSubsystem);
//     }

//     @Override
//     public void initialize() {
//     }

//     @Override
//     public void execute() {
//         // double armPosition = RobotContainer.armSubsystem.getArmPosition();
//         // if (armPosition >= 0.5) {
//         //     shooterSubsystem.runShooter(0.3, 0.3);
//         //     shooterSubsystem.intake(0.4);
//         // } else if (armPosition < 0.4) {
//         //     shooterSubsystem.stopShooter();
//         // }

//     }

//     @Override
//     public void end(boolean interrupted) {
        
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }

// }
