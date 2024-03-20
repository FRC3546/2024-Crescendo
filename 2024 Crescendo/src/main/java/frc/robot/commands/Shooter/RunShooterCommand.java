package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooterCommand extends Command{

    ShooterSubsystem shooterSubsystem;
    ClimbSubsystem climberSubsystem;

    DoubleSupplier upperShooterSpeed;
    DoubleSupplier lowerShooterSpeed;
    
    public RunShooterCommand(ShooterSubsystem shooterSubsystem, ClimbSubsystem climberSubsystem, DoubleSupplier upperShooterSpeed, DoubleSupplier lowerShooterSpeed){

        this.upperShooterSpeed = upperShooterSpeed;
        this.lowerShooterSpeed = lowerShooterSpeed;

        this.climberSubsystem = climberSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
        // System.out.println(upperShooterSpeed.getAsDouble() +" "+ lowerShooterSpeed.getAsDouble());
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute() {

        if(climberSubsystem.isClimberReleased()){
            shooterSubsystem.stopShooter();
        }

        else{
            shooterSubsystem.runShooter(upperShooterSpeed.getAsDouble(), lowerShooterSpeed.getAsDouble());
            // System.out.println(upperShooterSpeed.getAsDouble() +" "+ lowerShooterSpeed.getAsDouble());
        }
    }

    @Override
    public void end(boolean interrupted) {
        // System.out.println("command ended");
        shooterSubsystem.stopShooter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

