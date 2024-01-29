package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class PIDShooterCommand extends Command{

    IntakeShooterSubsystem shooterSubsystem;

    int RPM;
    
    public PIDShooterCommand(IntakeShooterSubsystem shooterSubsystem, int RPM){

        this.RPM = RPM;

        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        shooterSubsystem.runShooter(RPM);
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

