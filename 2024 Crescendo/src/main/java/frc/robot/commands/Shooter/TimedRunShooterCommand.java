package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import com.fasterxml.jackson.databind.ser.std.NumberSerializers.DoubleSerializer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class TimedRunShooterCommand extends Command{

    IntakeShooterSubsystem shooterSubsystem;
    Timer timer = new Timer();
    DoubleSupplier upperShooterSpeed;
    DoubleSupplier lowerShooterSpeed;
    double time;
    
    public TimedRunShooterCommand(IntakeShooterSubsystem shooterSubsystem, DoubleSupplier upperShooterSpeed, DoubleSupplier lowerShooterSpeed, double time){

        this.upperShooterSpeed = upperShooterSpeed;
        this.lowerShooterSpeed = lowerShooterSpeed;
        this.time = time;

        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
        System.out.println(upperShooterSpeed.getAsDouble() +" "+ lowerShooterSpeed.getAsDouble());
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        shooterSubsystem.runShooter(upperShooterSpeed.getAsDouble(), lowerShooterSpeed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return (timer.get() >= time);
    }
}

