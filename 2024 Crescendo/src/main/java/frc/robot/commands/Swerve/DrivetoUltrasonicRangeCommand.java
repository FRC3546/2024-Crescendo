package frc.robot.commands.Swerve;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auto.TimedDriveGyro;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class DrivetoUltrasonicRangeCommand extends Command {

    private double ySetpoint;

    private PIDController ypidLoop;
    private SwerveSubsystem swerveSubsystem;
    private LedSubsystem ledSubsystem;

    DoubleSupplier xTranslation;

    

  /**
   * Creates a new ExampleCommand.
   *
   * @param LimelightSubsystem The subsystem used by this command.
   */
  public DrivetoUltrasonicRangeCommand(SwerveSubsystem swerveSubsystem, LedSubsystem ledSubsystem, double ySetpoint) {

   this.ySetpoint = ySetpoint;
    
    ypidLoop = new PIDController(0.01, 0, 0);
    ypidLoop.setTolerance(3);
    
    ypidLoop.setSetpoint(ySetpoint);

    this.swerveSubsystem = swerveSubsystem;
    this.ledSubsystem = ledSubsystem;
    addRequirements(swerveSubsystem, ledSubsystem);
  }

  
  @Override
  public void initialize() {

    ledSubsystem.clearAnimation();
  }

  @Override
  public void execute() {

    // if(pidLoop.atSetpoint()){
    //   ledSubsystem.setStrobeAnimation(ledSubsystem.white, 0.5);
    // } else{
    //   ledSubsystem.clearAnimation();
    //   ledSubsystem.off();
    // }

    swerveSubsystem.driveFieldOrientedMaxVelocity(0, -ypidLoop.calculate(swerveSubsystem.getDistanceInches()), 0);

    // if(ypidLoop.atSetpoint() && xpidLoop.atSetpoint()){
    //     new TimedDriveGyro(swerveSubsystem, -0.5, 0, () -> swerveSubsystem.getHeading().getDegrees(), 1);
    // }
  }
  


  @Override 
  public void end(boolean interrupted) {
    ledSubsystem.clearAnimation();
    swerveSubsystem.driveCommand(() -> 0, () -> 0, () -> 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}