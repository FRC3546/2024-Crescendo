package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command {

    private final ClimbSubsystem m_climbSubsystem;

    public ClimbCommand(ClimbSubsystem climbSubsystem) {

        m_climbSubsystem = climbSubsystem;
        addRequirements(climbSubsystem);

    }

    @Override
    public void initialize() {
        
        m_climbSubsystem.rotateClimbers();

    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}