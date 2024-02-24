package frc.robot.Auto;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;


public class TimedDrive extends Command{

    Timer timer = new Timer();
    SwerveSubsystem swerveSubsystem;
    double vx;
    double vy;
    double vr;
    double time;
    
    public TimedDrive(SwerveSubsystem swerveSubsystem, double vx, double vy, double vr, double time){
        
        this.vx = vx;
        this.vy = vy;
        this.vr = vr;
        this.time = time;

        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);

    }

    @Override
    public void initialize(){

        timer.reset();
        timer.start();
        swerveSubsystem.drive(new ChassisSpeeds(vx,vy,vr));

    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
        swerveSubsystem.drive(new ChassisSpeeds(0,0,0));
    }

    @Override
    public boolean isFinished() {
        return (timer.get() >= time);
    }
}