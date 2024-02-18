package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class InputRunShooterCommand extends Command{

    IntakeShooterSubsystem shooterSubsystem;

    double upperShooterSpeed;
    double lowerShooterSpeed;
    
    public InputRunShooterCommand(IntakeShooterSubsystem shooterSubsystem){

        

        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
         shooterSubsystem.inputRunShooter();
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

