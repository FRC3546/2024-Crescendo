package frc.robot.commands.Climb;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class JoystickClimbCommand extends Command {

    private final ClimbSubsystem climbSubsystem;
    private DoubleSupplier speed;

    public JoystickClimbCommand(ClimbSubsystem climbSubsystem, DoubleSupplier speed) {

        this.climbSubsystem = climbSubsystem;
        this.speed = speed;
        addRequirements(climbSubsystem);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        climbSubsystem.rotateClimbers(speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.stopClimber();
    }

    @Override
    public boolean isFinished() {
        return climbSubsystem.getLimitSwitchValue();
    }
}