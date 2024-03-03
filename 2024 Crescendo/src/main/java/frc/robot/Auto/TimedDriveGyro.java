package frc.robot.Auto;
import java.util.function.DoubleSupplier;

import com.fasterxml.jackson.databind.ser.std.NumberSerializers.DoubleSerializer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;


public class TimedDriveGyro extends Command{

    PIDController pidLoop;
    Timer timer = new Timer();
    SwerveSubsystem swerveSubsystem;
    DoubleSupplier vr;
    double vx;
    double vy;
    double time;
    
    public TimedDriveGyro(SwerveSubsystem swerveSubsystem, double vx, double vy, DoubleSupplier vr, double time){
        
        pidLoop = new PIDController(0.12, 0.1, 0);
        pidLoop.setTolerance(0);

        pidLoop.setSetpoint(vr.getAsDouble());

        this.vr = vr;
        this.vx = vx;
        this.vy = vy;
        this.time = time;

        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);

    }

    @Override
    public void initialize(){

        timer.reset();
        timer.start();

    }

    @Override
    public void execute() {

        swerveSubsystem.drive(new ChassisSpeeds(vx,vy,pidLoop.calculate(swerveSubsystem.getHeading().getDegrees())));
    }

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