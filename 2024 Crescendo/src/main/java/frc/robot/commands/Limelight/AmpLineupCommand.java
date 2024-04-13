package frc.robot.commands.Limelight;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auto.TimedDriveGyro;
import frc.robot.commands.Swerve.DrivetoUltrasonicRangeCommand;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AmpLineupCommand extends Command {

    private double xSetpoint;
    private double ySetpoint;

    private LimelightSubsystem limelightSubsystem;
    PIDController xpidLoop;
    PIDController ypidLoop;
    SwerveSubsystem swerveSubsystem;
    private LedSubsystem ledSubsystem;

    DoubleSupplier xTranslation;
    DoubleSupplier yTranslation;

    

  /**
   * Creates a new ExampleCommand.
   *
   * @param LimelightSubsystem The subsystem used by this command.
   */
  public AmpLineupCommand(LimelightSubsystem limelightSubsystem, SwerveSubsystem swerveSubsystem, LedSubsystem ledSubsystem, double xSetpoint, double ySetpoint, DoubleSupplier rotation) {

   this.xSetpoint = xSetpoint;
   this.ySetpoint = ySetpoint;
    
    xpidLoop = new PIDController(0.05, 0, 0.01);
    xpidLoop.setTolerance(1);
    
    ypidLoop = new PIDController(0.05, 0, 0.01);
    ypidLoop.setTolerance(1);
    //0.03
    // 0
    // 2
    xpidLoop.setSetpoint(xSetpoint);
    ypidLoop.setSetpoint(ySetpoint);

    this.limelightSubsystem = limelightSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.ledSubsystem = ledSubsystem;
    addRequirements(swerveSubsystem, ledSubsystem);
  }

  
  @Override
  public void initialize() {

    ledSubsystem.clearAnimation();

    Optional<Alliance> ally = DriverStation.getAlliance();

    if(ally.isPresent()){
            if (ally.get() == Alliance.Red) {
              limelightSubsystem.setPipeline(2);
            }
            if (ally.get() == Alliance.Blue) {
                limelightSubsystem.setPipeline(1);
            }
        }

    else{
      System.err.println("No Alliance value for TargetOnTheMove!");
    }
  
  }

  @Override
  public void execute() {

    // if(pidLoop.atSetpoint()){
    //   ledSubsystem.setStrobeAnimation(ledSubsystem.white, 0.5);
    // } else{
    //   ledSubsystem.clearAnimation();
    //   ledSubsystem.off();
    // }

    swerveSubsystem.drive(new ChassisSpeeds(
      -ypidLoop.calculate(limelightSubsystem.getLimelightY()),
      -xpidLoop.calculate(limelightSubsystem.getLimelightX()), 
      0)); 
  }
  


  @Override 
  public void end(boolean interrupted) {
    ledSubsystem.clearAnimation();
    swerveSubsystem.driveCommand(() -> 0, () -> 0, () -> 0);
  }

  @Override
  public boolean isFinished() {
    return xpidLoop.atSetpoint() && ypidLoop.atSetpoint();
  }
}