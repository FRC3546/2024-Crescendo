package frc.robot.commands.Limelight;

import frc.robot.subsystems.ExampleSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TargetClimbing extends Command {

    private DoubleSupplier setPosition;
    private final LimelightSubsystem limelightSubsystem;
    PIDController pidLoop;
    SwerveSubsystem swerveSubsystem;
    double tagId;
    double chainAngle;

    /**
     * Creates a new ExampleCommand.
     *
     * @param LimelightSubsystem The subsystem used by this command.
     */
    public TargetClimbing(
            LimelightSubsystem limelightSubsystem,
            SwerveSubsystem swerveSubsystem,
            DoubleSupplier setPosition) {

        this.setPosition = setPosition;
        pidLoop = new PIDController(0.1, 0, 0);
        pidLoop.setTolerance(1);


        this.limelightSubsystem = limelightSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {

        tagId = limelightSubsystem.getTagId();

        if (tagId == 11){
            chainAngle = 0;
        } 
         if (tagId == 12){
            chainAngle = 0;
        }
         if (tagId == 13){
            chainAngle = 0;
        }
         if (tagId == 14){
            chainAngle = 0;
        }
         if (tagId == 15){
            chainAngle = 0;
        }
         if (tagId == 16){
            chainAngle = 0;
        }
      
        pidLoop.setSetpoint(chainAngle);
        
    }

    @Override
    public void execute() {

        swerveSubsystem.driveFieldOriented(new ChassisSpeeds(
                0,
                0,
                pidLoop.calculate(limelightSubsystem.getLimelightX())));

    }

    @Override
    public void end(boolean interrupted) {
        // swerveSubsystem.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
