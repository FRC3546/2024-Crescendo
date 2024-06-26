package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class SensorReverseIntakeCommand extends Command{

    IntakeSubsystem intakeSubsystem;
    
    public SensorReverseIntakeCommand(IntakeSubsystem intakeSubsystem){
        
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);

    }

    @Override
    public void initialize(){
        // intakeSubsystem.intake();
        intakeSubsystem.intake(-0.3);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        //new WaitCommand(0.1);
        intakeSubsystem.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return !intakeSubsystem.getSecondSensorValue();
    }
}

