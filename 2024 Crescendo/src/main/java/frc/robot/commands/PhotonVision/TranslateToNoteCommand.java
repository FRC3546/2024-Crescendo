package frc.robot.commands.PhotonVision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TranslateToNoteCommand extends Command {

    private double offsetAngle;
    PIDController pidLoop;

    PhotonVisionSubsystem visionSubsystem;
    SwerveSubsystem swerveSubsystem;

    public TranslateToNoteCommand(
            SwerveSubsystem swerveSubsystem,
            PhotonVisionSubsystem visionSubsystem,
            double offsetAngle) {
 
        this.offsetAngle = offsetAngle;
        pidLoop = new PIDController(0.04, 0, 0);
        pidLoop.setTolerance(0);

        pidLoop.setSetpoint(offsetAngle);

        this.swerveSubsystem = swerveSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        swerveSubsystem.drive(new ChassisSpeeds(0, pidLoop.calculate(visionSubsystem.getX()), 0));
        
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        // return pidLoop.atSetpoint();
        return false;
    }
}
