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

    

  /**
   * Creates a new ExampleCommand.
   *
   * @param LimelightSubsystem The subsystem used by this command.
   */
  public TargetOnTheMove(LimelightSubsystem limelightSubsystem, SwerveSubsystem swerveSubsystem, DoubleSupplier xTranslation, DoubleSupplier yTranslation, DoubleSupplier setPosition) {

    this.xTranslation = xTranslation;
    this.yTranslation = yTranslation;
    
    this.setPosition = setPosition;
    pidLoop = new PIDController(0.04, 0, 0);
    pidLoop.setTolerance(3);
    //0.03
    // 0
    // 2
    pidLoop.setSetpoint(setPosition.getAsDouble());

    this.limelightSubsystem = limelightSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  
  @Override
  public void initialize() {

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