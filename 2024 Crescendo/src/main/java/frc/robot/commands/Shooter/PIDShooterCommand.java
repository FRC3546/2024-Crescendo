package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeShooterSubsystem;
import java.lang.Thread;

public class PIDShooterCommand extends Command{

    IntakeShooterSubsystem shooterSubsystem;

    int RPM;
    
    public PIDShooterCommand(IntakeShooterSubsystem shooterSubsystem, int RPM){

        this.RPM = RPM;

        if(RPM > Constants.Shooter.maxShooterRPM){
            RPM = Constants.Shooter.maxShooterRPM;
        }

        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
         shooterSubsystem.runShooter(RPM);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        
        new WaitCommand(1.5);
        shooterSubsystem.stopShooter();
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.isShooterAtRPM(RPM);
    }
}

