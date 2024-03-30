package frc.robot.commands.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateToAngle extends Command {

    private DoubleSupplier targetAngle;
    PIDController pidLoop;
    SwerveSubsystem swerveSubsystem;
    

  /**
   * Creates a new ExampleCommand.
   *
   * @param LimelightSubsystem The subsystem used by this command.
   */
  public RotateToAngle(SwerveSubsystem swerveSubsystem, DoubleSupplier targetAngle) {
    
    this.targetAngle = targetAngle;
    pidLoop = new PIDController(0.08, 0.15, 0);
    pidLoop.setTolerance(0.3);

    pidLoop.setSetpoint(targetAngle.getAsDouble());

    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  
  @Override
  public void initialize() {}

  @Override
  public void execute() {

    swerveSubsystem.driveFieldOriented(new ChassisSpeeds(0,0, pidLoop.calculate(swerveSubsystem.getHeading().getDegrees())));

  }
  


  @Override 
  public void end(boolean interrupted) {
    swerveSubsystem.driveFieldOriented(new ChassisSpeeds(0,0,0));
  }

  @Override
  public boolean isFinished() {
    // return pidLoop.atSetpoint();
    return false;
  }
}
