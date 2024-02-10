package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class JoystickClimbCommand extends Command {

    private final ClimbSubsystem climbSubsystem;
    private double speed;

    public JoystickClimbCommand(ClimbSubsystem climbSubsystem, double speed) {

        this.climbSubsystem = climbSubsystem;
        this.speed = speed;
        addRequirements(climbSubsystem);

    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        climbSubsystem.rotateClimbers(speed);
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.stopClimber();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}