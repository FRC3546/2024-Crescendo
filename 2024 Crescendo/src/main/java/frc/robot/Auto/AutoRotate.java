package frc.robot.Auto;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;


public class AutoRotate extends Command{

    SwerveSubsystem swerveSubsystem;
    
    public AutoRotate(SwerveSubsystem swerveSubsystem){
        
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);

    }

    @Override
    public void initialize(){
        
        swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(0,0,0.5));

    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(0,0,0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}