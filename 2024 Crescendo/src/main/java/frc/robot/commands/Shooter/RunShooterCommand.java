package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class RunShooterCommand extends Command{

    IntakeShooterSubsystem shooterSubsystem;

    double upperShooterSpeed;
    double lowerShooterSpeed;
    
    public RunShooterCommand(IntakeShooterSubsystem shooterSubsystem, double upperShooterSpeed, double lowerShooterSpeed){

        this.upperShooterSpeed = upperShooterSpeed;
        this.lowerShooterSpeed = lowerShooterSpeed;

        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        shooterSubsystem.runShooter(upperShooterSpeed, lowerShooterSpeed);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

