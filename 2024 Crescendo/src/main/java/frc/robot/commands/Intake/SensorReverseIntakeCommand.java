package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class SensorReverseIntakeCommand extends Command{

    IntakeSubsystem intakeSubsystem;
    public static boolean doneIntaking = false;
    
    public SensorReverseIntakeCommand(IntakeSubsystem intakeSubsystem){
        
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);

    }

    @Override
    public void initialize(){
        // intakeSubsystem.intake();
        intakeSubsystem.intake(-0.18);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        new WaitCommand(0.1);
        intakeSubsystem.stopIntake();
        doneIntaking = true;
    }

    @Override
    public boolean isFinished() {
        return !intakeSubsystem.getSensorValue();
    }
}

