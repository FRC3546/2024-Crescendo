package frc.robot.commands.Limelight;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TargetOnTheMove extends Command {

    private DoubleSupplier setPosition;
    private LimelightSubsystem limelightSubsystem;
    PIDController pidLoop;
    SwerveSubsystem swerveSubsystem;

    DoubleSupplier xTranslation;
    DoubleSupplier yTranslation;

    Optional<Alliance> ally;
    

  /**
   * Creates a new ExampleCommand.
   *
   * @param LimelightSubsystem The subsystem used by this command.
   */
  public TargetOnTheMove(LimelightSubsystem limelightSubsystem, SwerveSubsystem swerveSubsystem, DoubleSupplier xTranslation, DoubleSupplier yTranslation, DoubleSupplier setPosition) {

    ally = DriverStation.getAlliance();
    this.xTranslation = xTranslation;
    this.yTranslation = yTranslation;
    
    this.setPosition = setPosition;
    pidLoop = new PIDController(0.06, 0, 0.3);
    pidLoop.setTolerance(0.5);

    pidLoop.setSetpoint(setPosition.getAsDouble());

    this.limelightSubsystem = limelightSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  
  @Override
  public void initialize() {

    if(ally.isPresent()){
            if (ally.get() == Alliance.Red) {
              limelightSubsystem.setPipeline(2);
            }
            if (ally.get() == Alliance.Blue) {
                limelightSubsystem.setPipeline(1);
            }
        }
  
  }

  @Override
  public void execute() {

    // System.out.println(xTranslation.getAsDouble() + " " + yTranslation.getAsDouble() + " " + setPosition.getAsDouble());
    swerveSubsystem.driveFieldOrientedMaxVelocity(xTranslation.getAsDouble(), yTranslation.getAsDouble(), pidLoop.calculate(limelightSubsystem.getLimelightX()));
    // swerveSubsystem.driveCommand(xTranslation, yTranslation, () -> 0);

  }
  


  @Override 
  public void end(boolean interrupted) {
    swerveSubsystem.driveCommand(() -> 0, () -> 0, () -> 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}